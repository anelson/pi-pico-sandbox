#![no_std]

mod bus;
mod keys;

use core::marker::PhantomData;
use core::num::NonZeroU8;

pub use bus::*;
pub use keys::*;

const INITIAL_DISPLAY_STATE: &[u8; 16] = &[
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
];

pub struct Tm1638Builder;

impl Tm1638Builder {
    /// Use an arbitrary [`BusDriver`] implementation; nothing more needs to be specified!
    pub fn with_bus_driver<D: BusDriver>(self, driver: D) -> Tm1638Builder3<D> {
        Tm1638Builder3 { driver }
    }

    /// Use an arbitrary [`Timer`] implementation with this bus driver.
    pub fn with_timer<T: Timer>(self) -> Tm1638Builder1<T> {
        Tm1638Builder1 {
            _timer: Default::default(),
        }
    }

    #[cfg(feature = "embassy-time")]
    /// Use the [`Timer`] implementation built using `embassy-time`
    pub fn with_embassy_timer(self) -> Tm1638Builder1<EmbassyTimeTimer> {
        self.with_timer::<EmbassyTimeTimer>()
    }
}

pub struct Tm1638Builder1<T: Timer> {
    _timer: PhantomData<T>,
}

impl<T: Timer> Tm1638Builder1<T> {
    /// Use the bit-banging driver, with an arbitrary implementation of [`Pins`] specific to your
    /// target platform
    pub fn with_bit_banging_driver<P: Pins>(self, pins: P) -> Tm1638Builder2<P, T> {
        Tm1638Builder2 {
            _timer: self._timer,
            pins,
        }
    }

    /// Use a bit-banging driver talking to the specified Embassy RP HAL pins
    #[cfg(feature = "embassy-rp")]
    pub fn with_embassy_rp_pins<
        'a,
        StrobePin: embassy_rp::gpio::Pin,
        ClockPin: embassy_rp::gpio::Pin,
        DioPin: embassy_rp::gpio::Pin,
    >(
        self,
        strobe: StrobePin,
        clock: ClockPin,
        dio: DioPin,
    ) -> Tm1638Builder2<EmbassyRpPins<'a, StrobePin, ClockPin, DioPin>, T> {
        self.with_bit_banging_driver(EmbassyRpPins::new(strobe, clock, dio))
    }
}

pub struct Tm1638Builder2<P: Pins, T: Timer> {
    pins: P,
    _timer: PhantomData<T>,
}

impl<P: Pins, T: Timer> Tm1638Builder2<P, T> {
    /// Construct the [`Tm1638`] instance using the bit-banging driver.
    ///
    /// This is fallible if the underling I/O implementation is.
    pub fn build(self) -> Result<Tm1638<BitBangingBusDriver<P, T>>, P::Error> {
        let driver = BitBangingBusDriver::new(self.pins)?;
        Ok(Tm1638::new(driver))
    }
}

pub struct Tm1638Builder3<D: BusDriver> {
    driver: D,
}

impl<D: BusDriver> Tm1638Builder3<D> {
    /// Construct the [`Tm1638`] instance using the selected driver.
    pub fn build(self) -> Tm1638<D> {
        Tm1638::new(self.driver)
    }
}

/// Driver for TM1638 display and switch controllers.
///
/// The implementation is generalized over the implementation of the underling bus protocol driver,
/// behind the [`BusDriver`] trait.  This allows most of the code to remain the same, while
/// supporting multiple hardware HALs and timer implementations.
///
/// The most straightforward way to instantiate this driver is using [`Self::builder`] which
/// returns a builder type with which you can get easy access to the built-in implementations.
///
/// For example, to use the `embassy-time` timer implementation and the `embassy-rp` HAL for
/// RP2040:
///
/// ```
/// # #[cfg(all(feature = "embassy-time", feature = "embassy-rp"))]
/// # {
/// let p = embassy_rp::init(Default::default());
/// let mut driver = tm1638::Tm1638::builder()
///     .with_embassy_timer()
///     .with_embassy_rp_pins(p.PIN_6, p.PIN_7, p.PIN_8)
///     .build()
///     .unwrap();
/// # }
/// ```
pub struct Tm1638<Driver> {
    driver: Driver,
    inc_addressing_mode: bool,
}

impl Tm1638<()> {
    /// Return a builder pattern implementation to ease some of the type parameter complexity
    /// around creating the bus driver and timer.
    ///
    /// This is not required; you can always instantiate the driver without a builder, but you
    /// might have to type more angle brackets to do so.
    pub fn builder() -> Tm1638Builder {
        Tm1638Builder
    }
}

impl<Driver: BusDriver> Tm1638<Driver> {
    pub fn new(driver: Driver) -> Self {
        Self {
            driver,
            inc_addressing_mode: false,
        }
    }

    /// Reset the TM1638 state, blanking all of the LEDs
    pub async fn init(&mut self) -> Result<(), Driver::Error> {
        self.blank_display().await?;

        // Activate the display with a reasonable default brightness
        self.activate_display(0x02).await
    }

    /// Blank the display state, including all 7 seg displays and LEDs
    pub async fn blank_display(&mut self) -> Result<(), Driver::Error> {
        // Put the controller in incremental addressing mode and initialize all 7-segment displays to be blank
        self.set_incrementing_addressing().await?;

        self.apply_write_command(WriteCommand::WriteMultipleChars {
            start_display_number: 0,
            segment_masks: INITIAL_DISPLAY_STATE,
        })
        .await
    }

    /// Activate or deactivate the display on the board.
    ///
    /// The brightness is a value from 0 (lowest brightness) to 7 (highest brightness)
    pub async fn activate_display(&mut self, brightness: u8) -> Result<(), Driver::Error> {
        self.apply_write_command(WriteCommand::ActivateDisplay { brightness })
            .await
    }

    /// Deactivate the display
    pub async fn deactivate_display(&mut self) -> Result<(), Driver::Error> {
        self.apply_write_command(WriteCommand::DeactivateDisplay)
            .await
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
    pub async fn set_display_mask(&mut self, address: u8, mask: u8) -> Result<(), Driver::Error> {
        self.set_fixed_addressing().await?;

        self.apply_write_command(WriteCommand::WriteSingleChar {
            display_number: address,
            segment_mask: mask,
        })
        .await
    }

    /// Set the mask for an LED output for a specific LED by `address`.
    ///
    /// `0` is the left-most LED on the board I have; `7` is the right-most.
    ///
    /// `mask` controls the SEGMENT9 and SEGMENT10 outputs that for the GRID pin correspondign to
    /// `address`.  Therefore only the two least significant bits of `mask` are used.  I have read
    /// that some boards with the TM1638 chip have multi-color LEDs in which case the mask might be
    /// used to control the color.  On my board the LEDs are all red so this isn't applicable.
    pub async fn set_led_mask(&mut self, address: u8, mask: u8) -> Result<(), Driver::Error> {
        self.set_fixed_addressing().await?;

        self.apply_write_command(WriteCommand::WriteLed {
            led_number: address,
            mask,
        })
        .await
    }

    /// Read the raw key bitmask from the controller
    pub async fn read_keys(&mut self) -> Result<Keys, Driver::Error> {
        let mut buffer = [0u8; keys::KEY_BYTES];

        self.apply_read_command(ReadCommand::ReadKeys, &mut buffer)
            .await?;

        #[cfg(feature = "defmt")]
        defmt::trace!("keys = {:?}", buffer);

        Ok(Keys::new(buffer))
    }

    /// Switch the chip to fixed addressing mode, so all write operations to whatever the current
    /// address is, with no automatic incrementing of the address.
    async fn set_fixed_addressing(&mut self) -> Result<(), Driver::Error> {
        if self.inc_addressing_mode {
            self.apply_write_command(WriteCommand::SetFixedDisplayAddressing)
                .await?;
            self.inc_addressing_mode = false;
        }

        Ok(())
    }

    /// Switch the chip to incremental addressing mode, so each byte written will automatically
    /// increment the current address by one byte, which makes it much more convenient to write
    /// to multiple LEDs and 7-seg displays.
    async fn set_incrementing_addressing(&mut self) -> Result<(), Driver::Error> {
        if !self.inc_addressing_mode {
            self.apply_write_command(WriteCommand::SetIncrementalDisplayAddressing)
                .await?;
            self.inc_addressing_mode = true;
        }

        Ok(())
    }

    /// Apply the command to the controller
    async fn apply_write_command<'c>(
        &mut self,
        command: WriteCommand<'c>,
    ) -> Result<(), Driver::Error> {
        let (command_byte, data_bytes) = command.encode();

        #[cfg(feature = "defmt")]
        defmt::trace!("command byte = {=u8:x}", command_byte);

        if let Some(data_bytes) = data_bytes {
            self.send_command_and_data_bytes(command_byte, data_bytes)
                .await?;
        } else {
            self.send_command_byte(command_byte).await?;
        }

        Ok(())
    }

    async fn apply_read_command(
        &mut self,
        command: ReadCommand,
        read_buffer: &mut [u8],
    ) -> Result<(), Driver::Error> {
        let (command_byte, read_bytes) = command.encode();

        #[cfg(feature = "defmt")]
        defmt::trace!("command byte = {=u8:x}", command_byte);

        #[cfg(feature = "defmt")]
        defmt::debug_assert!(read_bytes.get() as usize <= read_buffer.len());

        // Limit the read buffer to just the range needed to store these results
        let read_buffer = &mut read_buffer[0..read_bytes.get() as usize];

        self.send_command_byte_and_read(command_byte, read_buffer)
            .await
    }

    /// Send a single byte that represents a command, so strobe will be pulled low
    /// before the command's bits are sent, and then pulled high again after.
    async fn send_command_byte(&mut self, b: u8) -> Result<(), Driver::Error> {
        self.driver.send_command(b).await
    }

    /// Send a single byte that represents a command followed by one or more data bytes, so strobe
    /// will be pulled low before the command's bits are sent, and not pulled high again
    /// until after the data bytes are sent.
    async fn send_command_and_data_bytes(
        &mut self,
        cmd: u8,
        data: &[u8],
    ) -> Result<(), Driver::Error> {
        self.driver.send_command_write_data(cmd, data).await
    }

    /// Send a single byte that represents a command and which expects a response back from the
    /// controller, so strobe will be pulled low before the command's bits are sent, and then pulled high again after all bytes are read.
    async fn send_command_byte_and_read(
        &mut self,
        b: u8,
        read_buffer: &mut [u8],
    ) -> Result<(), Driver::Error> {
        self.driver.send_command_read_data(b, read_buffer).await
    }
}

/// Represents possible write-only commands sent to the TM1638 as Rust enums for greater readability.
enum WriteCommand<'a> {
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

impl<'a> WriteCommand<'a> {
    /// Convert this command into the appropriate byte sequence to send to the controller.
    ///
    /// Return value is a tuple consisting of the following:
    ///
    /// - Command byte to send to controller
    /// - (Optional) slice of data bytes to send along with command byte
    ///
    /// The command byte and data bytes (if any) are sent together, during a single interval in which the strobe pin is pulled low.
    fn encode<'me>(&'me self) -> (u8, Option<&'me [u8]>)
    where
        'a: 'me,
    {
        match self {
            WriteCommand::ActivateDisplay { brightness } => {
                // According to the data sheet, "display on" is just a command byte
                // The lowest three bits indicate the brightness.
                //
                // See 5.3 in the data sheet
                #[cfg(feature = "defmt")]
                defmt::debug_assert!(*brightness < 0b1000);
                let brightness = brightness & 0b0000_0111;

                (0b1000_1000 | brightness, None)
            }
            WriteCommand::DeactivateDisplay => {
                // Similar to activation, deactivation is also a single byte
                (0b1000_0000, None)
            }
            WriteCommand::SetIncrementalDisplayAddressing => {
                // Per section 5.1 in the data sheet
                (0b0100_0000, None)
            }
            WriteCommand::SetFixedDisplayAddressing => {
                // Per section 5.1 in the data sheet
                (0b0100_0100, None)
            }
            WriteCommand::WriteLed { led_number, mask } => {
                // Section 5.2 in the data sheet shows how to set the address
                // Section 9.2 shows how addressing works in fixed mode.  9.1 shows incremental
                // mode
                //
                // The 7 segments displays are at odd numbered offsets (ie, first LED is at
                // byte 1, second display is at byte 3).  This is because the LEDs and the
                // 7-segment displays share the same address range
                #[cfg(feature = "defmt")]
                defmt::debug_assert!(*led_number < 0b1_0000);
                (
                    0b1100_0000 | (((led_number << 1) + 1) & 0b0000_1111),
                    Some(core::slice::from_ref(mask)),
                )
            }
            WriteCommand::WriteSingleChar {
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
                #[cfg(feature = "defmt")]
                defmt::debug_assert!(*display_number < 0b1_0000);
                (
                    0b1100_0000 | ((display_number << 1) & 0b0000_1111),
                    Some(core::slice::from_ref(segment_mask)),
                )
            }
            WriteCommand::WriteMultipleChars {
                start_display_number,
                segment_masks,
            } => {
                // This works just like the `WriteSingleChar` command, except we assume the
                // controller is in incremental addressing mode, and send multiple bytes after the
                // address command
                #[cfg(feature = "defmt")]
                defmt::debug_assert!(*start_display_number < 0b1_0000);
                #[cfg(feature = "defmt")]
                defmt::debug_assert!(*start_display_number as usize + segment_masks.len() <= 16);
                (
                    0b1100_0000 | (start_display_number & 0b0000_1111),
                    Some(segment_masks),
                )
            }
        }
    }
}

/// Represents possible read commands sent to the TM1638 which read data from the controller
enum ReadCommand {
    /// Request the controller to send four bytes of key scanning data reflecting current state of
    /// keys
    ReadKeys,
}

impl ReadCommand {
    /// Convert this command into the appropriate byte sequence to send to the controller.
    ///
    /// Return value is a tuple consisting of the following:
    ///
    /// - Command byte to send to controller
    /// - number of bytes to read from the controller after command is sent.
    ///
    /// The command byte and response bytes (if any) are read, during a single interval in which
    /// the strobe pin is pulled low.
    fn encode(&self) -> (u8, NonZeroU8) {
        match self {
            ReadCommand::ReadKeys => {
                // SHITTY: `unwrap` on `Option` isn't yet a const fn in stable Rust as of this
                // writing, so `unsafe` has to be used here.  It's not actually unsafe since we can
                // clearly see that `4` is not `0`.
                const KEY_BYTES: NonZeroU8 =
                    unsafe { NonZeroU8::new_unchecked(keys::KEY_BYTES as u8) };

                (0b0100_0010, KEY_BYTES)
            }
        }
    }
}

#[cfg(test)]
mod test {
    #[test]
    fn foo() {}
}
