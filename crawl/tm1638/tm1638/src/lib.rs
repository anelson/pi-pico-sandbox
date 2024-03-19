#![no_std]

mod bus;
#[cfg(feature = "ascii-font")]
mod font;
mod keys;

use core::marker::PhantomData;
use core::num::NonZeroU8;
use core::num::NonZeroUsize;

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

        self.apply_write_command(WriteCommand::WriteDisplayBytes {
            start_display_address: 0,
            display_data: INITIAL_DISPLAY_STATE,
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

    /// Set the value of SEG1-SEG8 for a given GRID value.
    ///
    /// The datasheet in section 7 has a diagram showing the recommended mapping of `SEG` output
    /// pins to actual segments in the display.  This mapping is used on the MDU1093 board that I
    /// tested with.  But obviously another implementation is free to wire things up differently.
    ///
    /// On the MDU1093 board and maybe others, these pins are wired to 7-segment displays, one
    /// digit for each of 8 GRID pins.  Other implementations might use all 10 SEG pins to control
    /// a 9 segment display or some other output device, in which case use [`set_grid_upper_byte`]
    /// or the lower level [`write_display_data`] functions.
    ///
    /// Every bit in `value` corresponds to a SEG pin.  Bit 0 is SEG1, bit 1 is SEG2, etc.
    pub async fn set_grid_lower_byte(&mut self, grid: u8, value: u8) -> Result<(), Driver::Error> {
        self.set_fixed_addressing().await?;

        self.apply_write_command(WriteCommand::WriteDisplayByte {
            // Translate the address of the 7-segment display to a memory address
            // Between each 7-seg display there's a byte for controlling the SEG9 and SEG10 pins, so to
            // translate this we multiply the address by 2
            display_address: grid * 2,
            display_data: value,
        })
        .await
    }

    /// Set the value of SEG9-SEG10 for a given GRID value.
    ///
    /// Only the lower two bits of `value` are used.
    ///
    /// On the MDU1093 board, SEG9 is connected to a separate red LED For each GRID pin.
    /// Supposedly there are other boards that use both SEG9 and SEG10 on multi-color LEDs to allow
    /// control of the LED color.  Other applications could use these pins for anything else.
    pub async fn set_grid_upper_byte(&mut self, grid: u8, value: u8) -> Result<(), Driver::Error> {
        self.set_fixed_addressing().await?;

        self.apply_write_command(WriteCommand::WriteDisplayByte {
            // We want to control the SEG9 and SEG10 pins for the specified GRID pin, so we need to
            // multiple the addres by 2 and add 1
            display_address: grid * 2 + 1,
            display_data: value,
        })
        .await
    }

    /// Set the values of one or more sets of SEG1-SEG8 pins connected to contiguous GRID pins.
    ///
    /// Each byte in `digits` is a bitmask, where each bit corresponds to a segment pin from SEG1
    /// to SEG8.
    /// For the purposes of this function, bit N of each byte in `digits` corresponds to the SEGN
    /// bit on the controller.
    ///
    /// `grid` should be 0 to start the output on the first digit (GRID1), 1 to start at
    /// the second digit (GRID2), etc.
    ///
    /// This is just a convenience function for calling [`set_grid_upper_byte`] for multiple GRID
    /// values at a time.
    pub async fn set_grid_lower_bytes(
        &mut self,
        grid: u8,
        values: &[u8],
    ) -> Result<(), Driver::Error> {
        defmt::debug_assert!(grid < 8);

        // Recall from the datasheet that the address of the SEG1-SEG8 pins is right before the
        // address for the SEG9 and SEG10 pins for the same GRID pin.  This means that if you want
        // to write to multiple 7-segment displays connected to SEG1-SEG8, and not to separate LEDs
        // connected to SEG9 and SEG10 (on the MDU1093 board SEG9 is a red LED and SEG10 is not
        // used), you have to skip every other byte.
        //
        // For this reason WriteMultipleChars is not very useful except to clear the entire display
        // at the beginning.
        self.set_fixed_addressing().await?;

        for (index, digit) in values.into_iter().enumerate() {
            self.set_grid_lower_byte(grid + index as u8, *digit).await?;
        }

        Ok(())
    }

    /// Write some short text to the SEG1-SEG8 pins of multiple GRID pins, assuming that those SEG
    /// pins are connected to 7-segment displays according to the recommentation in the data sheet.
    ///
    /// Renders ASCII characters to a primitive kind of font that will be sort of readable on a 7
    /// segment display.
    ///
    /// Use `max_chars` to render just part of the display memory.
    #[cfg(feature = "ascii-font")]
    pub async fn set_grid_lower_bytes_text(
        &mut self,
        start_display_address: u8,
        max_chars: Option<NonZeroUsize>,
        text: &str,
    ) -> Result<(), Driver::Error> {
        // The controller can only drive up to 8 7-seg displays, so we need no more than 8 bytes
        // for this.

        let mut bytes = [0u8; 8];

        font::render_text(text, &mut bytes[..]);

        // Clamp the rendered text if needed
        let data = if let Some(max_chars) = max_chars {
            &bytes[0..max_chars.get()]
        } else {
            &bytes[..]
        };

        self.set_grid_lower_bytes(start_display_address, data).await
    }

    /// Write multiple bytes of arbitrary data to the display memory.
    ///
    /// Unlike the helper functions [`set_grid_lower_byte`] and [`set_grid_upper_byte`], this does
    /// not perform any translation of the data being written.  Writes will start at
    /// `start_display_address` and be written contiguously to the controller's display memory for
    /// `data.len()` bytes.  Due to the layout of the display memory, this will control SEG1-SEG8
    /// pins or SEG9-SEG10 pins or both, depending on the address.
    ///
    /// Refer to the map of display memory in section 7 of the data sheet to understand how this
    /// will actually work.
    ///
    /// This is considerably more efficient than writing a byte at a time, at the expense of being
    /// more complex for the caller to manage.
    pub async fn write_display_data(
        &mut self,
        start_display_address: u8,
        data: &[u8],
    ) -> Result<(), Driver::Error> {
        defmt::debug_assert!(16 < 8);

        // There's an optimized scheme for "bulk" transfer of data to the controller, but it has to
        // be activated before we perform the write.
        self.set_incrementing_addressing().await?;

        self.apply_write_command(WriteCommand::WriteDisplayBytes {
            start_display_address,
            display_data: data,
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

    /// Write a single byte to an address in display memory.
    ///
    /// See section 6 of the data sheet for a map of the display memory and what combination of
    /// GRID and SEG pins each bit of that memory corresponds to.
    ///
    /// In incremental write mode, after writing a byte the display address is automatically
    /// incremented.  In fixed write mode, all written data goes to this same address until changed
    /// This command can be used in either mode.
    WriteDisplayByte {
        /// The display address to write to.
        display_address: u8,

        /// The byte to write to this address.  This will control the value of either the SEG1-SEG8
        /// pins, or the SEG9-SEG10 pins, and some GRID pin, depending upon what display address
        /// was written
        display_data: u8,
    },
    /// Write multiple bytes to display memory, at a specific start address
    ///
    /// See section 6 of the data sheet for a map of the display memory and what combination of
    /// GRID and SEG pins each bit of that memory corresponds to.
    ///
    /// In incremental write mode, after writing a byte the display address is automatically
    /// incremented.
    /// Use this only with incremental display addressing mode.
    WriteDisplayBytes {
        /// The starting address to write to.
        /// Since this is valid only with incremental addressing mode, each byte is written to the
        /// next display in order, wrapping around to 0
        start_display_address: u8,

        /// Segment masks for the display(s) being written to.
        display_data: &'a [u8],
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
            WriteCommand::WriteDisplayByte {
                display_address,
                display_data,
            } => {
                // Section 5.2 in the data sheet shows how to set the address
                // Section 9.2 shows how addressing works in fixed mode.  9.1 shows incremental
                // mode
                #[cfg(feature = "defmt")]
                defmt::debug_assert!(*display_address < 0b1_0000);
                (
                    0b1100_0000 | (display_address & 0b0000_1111),
                    Some(core::slice::from_ref(display_data)),
                )
            }
            WriteCommand::WriteDisplayBytes {
                start_display_address: start_display_number,
                display_data: segment_masks,
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
