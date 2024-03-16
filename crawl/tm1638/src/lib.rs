#![no_std]

use defmt::*;
use either::Either;
use embassy_executor::Spawner;
use embassy_rp::{
    self, gpio,
    i2c::{self, Config},
};
use embassy_time::{Duration, Timer};
use embedded_hal_1::{
    digital::{OutputPin, PinState},
    i2c::I2c,
};
use {defmt_rtt as _, panic_probe as _};

/// Use a 1uS clock tick to ensure the TM1638 picks up the value
const CLOCK_TICK: Duration = Duration::from_micros(1);

/// Incremental write
const TM_WRITE_INC: u8 = 0x40;
/// leftmost segment Address C0 C2 C4 C6 C8 CA CC CE
const TM_SEG_ADR: u8 = 0xC0;
/// Start up device
const TM_ACTIVATE: u8 = 0x8F;

/// Brightness address
const TM_BRIGHT_ADR: u8 = 0x88;

/// Brightness mask
const TM_BRIGHT_MASK: u8 = 0x07;
/// Brightness can be 0x00 to 0x07 , 0x00 is least bright
const TM_DEFAULT_BRIGHTNESS: u8 = 0x02;

const INITIAL_DISPLAY_STATE: &[u8; 16] = &[
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
];

pub struct Tm1638<'a, StrobePin: gpio::Pin, ClockPin: gpio::Pin, DioPin: gpio::Pin> {
    strobe: gpio::Output<'a, StrobePin>,
    clock: gpio::Output<'a, ClockPin>,
    dio: gpio::Flex<'a, DioPin>,
    inc_addressing_mode: bool,
}

impl<'a, StrobePin: gpio::Pin, ClockPin: gpio::Pin, DioPin: gpio::Pin>
    Tm1638<'a, StrobePin, ClockPin, DioPin>
{
    pub fn new(strobe: StrobePin, clock: ClockPin, dio: DioPin) -> Self {
        Self {
            strobe: gpio::Output::new(strobe, gpio::Level::High),
            clock: gpio::Output::new(clock, gpio::Level::Low),
            dio: gpio::Flex::new(dio),
            inc_addressing_mode: false,
        }
    }

    /// Reset the TM1638 state, blanking all of the LEDs
    pub async fn init(&mut self) {
        // Initially the DIO pin is for output, except when scanning for key presses
        self.dio.set_as_output();

        // Set the pins in their initial conditions, corresponding to an idle state
        self.dio.set_low();

        // Put the controller in incremental addressing mode and initialize all 7-segment displays to be blank
        self.set_incrementing_addressing().await;

        self.apply_command(Command::WriteMultipleChars {
            start_display_number: 0,
            segment_masks: INITIAL_DISPLAY_STATE,
        })
        .await;

        // Activate the display with a reasonable default brightness
        self.activate_display(0x02).await;
    }

    /// Activate or deactivate the display on the board.
    ///
    /// The brightness is a value from 0 (lowest brightness) to 7 (highest brightness)
    pub async fn activate_display(&mut self, brightness: u8) {
        self.apply_command(Command::ActivateDisplay { brightness })
            .await;
    }

    /// Deactivate the display
    pub async fn deactivate_display(&mut self) {
        self.apply_command(Command::DeactivateDisplay).await;
    }

    /// Set a specific 7-segment display identified by `address` (`0` is the left-most display, max
    /// value is 7), to the value `mask`.
    ///
    /// `mask` is a bitmask in which the least significant 7 bits correspond to segments on the
    /// display, and the most significant bit corresponds to the `.` in the bottom right of the
    /// display.
    ///
    /// This uses the relative addressing mode of the TM1638.  To replace the entire contents of
    /// the display use `[XXX]`.
    pub async fn set_display_mask(&mut self, address: u8, mask: u8) {
        self.set_fixed_addressing().await;

        self.apply_command(Command::WriteSingleChar {
            display_number: address,
            segment_mask: mask,
        })
        .await;
    }

    /// Set the mask for an LED output for a specific LED by `address`.
    ///
    /// `0` is the left-most LED on the board I have; `7` is the right-most.
    ///
    /// `mask` controls the SEGMENT9 and SEGMENT10 outputs that for the GRID pin correspondign to
    /// `address`.  Therefore only the two least significant bits of `mask` are used.  I have read
    /// that some boards with the TM1638 chip have multi-color LEDs in which case the mask might be
    /// used to control the color.  On my board the LEDs are all red so this isn't applicable.
    pub async fn set_led_mask(&mut self, address: u8, mask: u8) {
        self.set_fixed_addressing().await;

        self.apply_command(Command::WriteLed {
            led_number: address,
            mask,
        })
        .await;
    }

    /// Switch the chip to fixed addressing mode, so all write operations to whatever the current
    /// address is, with no automatic incrementing of the address.
    async fn set_fixed_addressing(&mut self) {
        if self.inc_addressing_mode {
            self.apply_command(Command::SetFixedDisplayAddressing).await;
            self.inc_addressing_mode = false;
        }
    }

    /// Switch the chip to incremental addressing mode, so each byte written will automatically
    /// increment the current address by one byte, which makes it much more convenient to write
    /// to multiple LEDs and 7-seg displays.
    async fn set_incrementing_addressing(&mut self) {
        if !self.inc_addressing_mode {
            self.apply_command(Command::SetIncrementalDisplayAddressing)
                .await;
            self.inc_addressing_mode = false;
        }
    }

    async fn apply_command<'c>(&mut self, command: Command<'c>) {
        let (command_byte, data_bytes) = command.encode();

        debug!("command byte = {=u8:x}", command_byte);

        if let Some(data_bytes) = data_bytes {
            self.send_command_and_data_bytes(command_byte, data_bytes)
                .await;
        } else {
            self.send_command_byte(command_byte).await;
        }
    }

    /// Send a single byte that represents a command, so strobe will be pulled low
    /// before the command's bits are sent, and then pulled high again after.
    async fn send_command_byte(&mut self, b: u8) {
        self.strobe.set_low();
        self.send_byte(b).await;
        self.strobe.set_high();
    }

    /// Send a single byte that represents a command followed by one or more data bytes, so strobe
    /// will be pulled low before the command's bits are sent, and not pulled high again
    /// until after the data bytes are sent.
    async fn send_command_and_data_bytes(&mut self, cmd: u8, data: &[u8]) {
        defmt::debug_assert!(!data.is_empty());
        self.strobe.set_low();
        self.send_byte(cmd).await;
        for b in data {
            debug!("data byte = {=u8:x}", b);
            self.send_byte(*b).await;
        }
        self.strobe.set_high();
    }

    /// Send a single byte, maybe command maybe data this low level function doesn't care.
    ///
    /// Sends a bit at a time on the DIO pin
    async fn send_byte(&mut self, b: u8) {
        self.shift_byte_out(b).await;
    }

    /// Shift the byte value out on the DIO pin, LSB first, waiting an appropriate
    /// period of time between clock pulses to ensure the TM1638 can keep up
    async fn shift_byte_out(&mut self, b: u8) {
        for bit in 0..8 {
            let mask = 1 << bit;
            let value = (b & mask) != 0;

            self.dio.set_state(value.into()).unwrap();

            // XXX Experiment: the LA sometimes shows the clock going high within a few nano of the
            // DIO pin going high.  It's not clear if the chip will read this as a 0 or a 1.  Try
            // waiting a tick for the DIO pin to assume its state before bringing clock high.  The
            // datasheet says data is read from DIO on the rising edge of CLK so this detail
            // matters.
            Timer::after(CLOCK_TICK).await;

            // TODO: when do we set the pin as output?  Is this inefficient?
            //if bit == 0 {
            //    self.dio.set_as_output();
            //}

            self.clock.set_high();
            Timer::after(CLOCK_TICK).await;
            self.clock.set_low();
            Timer::after(CLOCK_TICK).await;
        }
    }
}

/// Represents possible commands sent to the TM1638 as Rust enums for greater readability.
enum Command<'a> {
    /// Turn on the display.  I think but am not sure that if the display had some previous
    /// contents they are displayed now
    ActivateDisplay {
        /// Brightness, in a range from 0 to 7.  If `brightness` is larger than this range it will
        /// be set to the value `brightness` mod 8.
        brightness: u8,
    },

    /// Deactivate the display.  I think but am not sure that the contents of the display are not
    /// lost they're still buffered in memory on the controller.
    DeactivateDisplay,

    /// Set the display addressing mode to incremental, so specific 7-segment displays can be
    /// targeted directly, with the target address automatically incrementing after each write.
    SetIncrementalDisplayAddressing,

    /// Set the display addressing mode to fixed, so specific 7-segment displays can be
    /// targeted directly, with the target address remaining fixed unless explicitly changed.
    SetFixedDisplayAddressing,

    /// Set the address of the display that will be written to and set the segment enablement mask
    /// for that display.
    /// In incremental write mode, after writing a byte the display address is automatically
    /// incremented.  In fixed write mode, all written data goes to this same address until changed
    /// This command can be used in either mode.
    WriteLed {
        /// The LED to write to.  There are 8 LEDs on the board I have
        led_number: u8,

        /// The bit mask controlling which segments on the display are illuminated
        /// According to the data sheet, only the two least significant bits are used to control
        /// the LED.
        mask: u8,
    },
    /// Set the address of the display that will be written to and set the segment enablement mask
    /// for that display.
    /// In incremental write mode, after writing a byte the display address is automatically
    /// incremented.  In fixed write mode, all written data goes to this same address until changed
    /// This command can be used in either mode.
    WriteSingleChar {
        /// The initial number of the segment to write to.
        display_number: u8,

        /// The bit mask controlling which segments on the display are illuminated
        segment_mask: u8,
    },
    /// Set the address of the display that will be written to, and write one or more segment
    /// enablement masks.
    /// In incremental write mode, after writing a byte the display address is automatically
    /// incremented.
    /// Use this only with incremental display addressing mode.
    WriteMultipleChars {
        /// The number of the first display to write to.  So a value of `1` means the first segment, `2` the second segment, etc.
        ///
        /// Since this is valid only with incremental addressing mode, each byte is written to the
        /// next display in order, wrapping around to 1
        start_display_number: u8,

        /// Segment masks for the display(s) being written to.
        segment_masks: &'a [u8],
    },
}

impl<'a> Command<'a> {
    /// Convert this command into the appropriate byte sequence to send to the controller.
    ///
    /// Return value is either a single command byte, or a command byte and one or more
    /// data bytes.
    ///
    /// The command byte and data bytes (if any) are sent together, during a single interval in
    /// which the strobe pin is pulled low.
    fn encode<'me>(&'me self) -> (u8, Option<&'me [u8]>)
    where
        'a: 'me,
    {
        match self {
            Command::ActivateDisplay { brightness } => {
                // According to the data sheet, "display on" is just a command byte
                // The lowest three bits indicate the brightness.
                //
                // See 5.3 in the data sheet
                defmt::debug_assert!(*brightness < 0b1000);
                let brightness = brightness & 0b0000_0111;

                (0b1000_1000 | brightness, None)
            }
            Command::DeactivateDisplay => {
                // Similar to activation, deactivation is also a single byte
                (0b1000_0000, None)
            }
            Command::SetIncrementalDisplayAddressing => {
                // Per section 5.1 in the data sheet
                (0b0100_0000, None)
            }
            Command::SetFixedDisplayAddressing => {
                // Per section 5.1 in the data sheet
                (0b0100_0100, None)
            }
            Command::WriteLed { led_number, mask } => {
                // Section 5.2 in the data sheet shows how to set the address
                // Section 9.2 shows how addressing works in fixed mode.  9.1 shows incremental
                // mode
                //
                // The 7 segments displays are at odd numbered offsets (ie, first LED is at
                // byte 1, second display is at byte 3).  This is because the LEDs and the
                // 7-segment displays share the same address range
                defmt::debug_assert!(*led_number < 0b1_0000);
                (
                    0b1100_0000 | (((led_number << 1) + 1) & 0b0000_1111),
                    Some(core::slice::from_ref(mask)),
                )
            }
            Command::WriteSingleChar {
                display_number,
                segment_mask,
            } => {
                // Section 5.2 in the data sheet shows how to set the address
                // Section 9.2 shows how addressing works in fixed mode.  9.1 shows incremental
                // mode
                //
                // The 7 segments displays are at even numbered offsets (ie, first display is at
                // byte 0, second display is at byte 2).  The odd numbered offsets set the
                // SEGMENT9 and SEGMENT10 pins which control the LEDs on the board.
                defmt::debug_assert!(*display_number < 0b1_0000);
                (
                    0b1100_0000 | ((display_number << 1) & 0b0000_1111),
                    Some(core::slice::from_ref(segment_mask)),
                )
            }
            Command::WriteMultipleChars {
                start_display_number,
                segment_masks,
            } => {
                // This works just like the `WriteSingleChar` command, except we assume the
                // controller is in incremental addressing mode, and send multiple bytes after the
                // address command
                defmt::debug_assert!(*start_display_number < 0b1_0000);
                defmt::debug_assert!(*start_display_number as usize + segment_masks.len() <= 16);
                (
                    0b1100_0000 | (start_display_number & 0b0000_1111),
                    Some(segment_masks),
                )
            }
        }
    }
}
