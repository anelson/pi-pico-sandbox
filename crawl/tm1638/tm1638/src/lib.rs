#![no_std]

mod bus;

use core::num::NonZeroU8;
use defmt::*;

pub use bus::*;

/// The number of bytes used to represent the state of the keys on the board
const KEY_BYTES: usize = 4;

const INITIAL_DISPLAY_STATE: &[u8; 16] = &[
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
];

pub struct Tm1638<Driver: BusDriver> {
    driver: Driver,
    inc_addressing_mode: bool,
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
        let mut buffer = [0u8; KEY_BYTES];

        self.apply_read_command(ReadCommand::ReadKeys, &mut buffer)
            .await?;

        trace!("keys = {:?}", buffer);

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

        trace!("command byte = {=u8:x}", command_byte);

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

        trace!("command byte = {=u8:x}", command_byte);

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

/// The state of the keys on the TM1638 board in response to a call to [`Tm1338::read_keys`]
///
/// According to the datasheet, the controller supports a keypad arraged in a 3x8 matrix.  This
/// struct makes it more egonomic to work with that matrix.
#[derive(Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Keys([u8; KEY_BYTES]);

impl Keys {
    fn new(bytes: [u8; KEY_BYTES]) -> Self {
        Self(bytes)
    }

    /// Quickly check if *any* keys are pressed
    pub fn any_pressed(&self) -> bool {
        self.0.iter().any(|byte| *byte != 0)
    }

    /// Test if the given key (identified by its column and row) is pressed
    pub fn is_pressed(&self, col: KeyColumn, row: KeyRow) -> bool {
        let nibble = row.extract_row_nibble_from_bytes(&self.0);
        debug!("nibble={=u8}", nibble);

        col.extract_value_from_row_nibble(nibble)
    }

    /// If at least one key is indicated as pressed, return that key's column and row, and update
    /// the key state so that key is no longer indicated as pressed.
    ///
    /// There are no guarantees regarding the order in which keys are returned in the case of multiple key presses.
    fn pop_key(&mut self) -> Option<(KeyColumn, KeyRow)> {
        // Find a non-empty byte, looking only at the bits that correspond to supported columns
        // (that means that bit 3 and bit 7 (0-based) are ignored; see the data sheet section 8 to
        // understand why if it's not obvious)
        if let Some((index, byte)) = self
            .0
            .iter_mut()
            .enumerate()
            .find(|(_index, byte)| (**byte & 0b0111_0111) != 0)
        {
            // we know that this byte is non-zero.  find the least significant bit that is set
            let set_bit = byte.trailing_zeros();

            if let Some(key_coords) = Self::bit_to_col_and_row(index as u8, set_bit as u8) {
                // Clear this bit before returning
                *byte &= !(1 << set_bit);

                Some(key_coords)
            } else {
                // This is a bug in the code; it shouldn't be possible that we get an invalid index
                // and set bit combo.  But handle this gracefully except in debug mode
                defmt::debug_assert!(
                    false,
                    "BUG: index={} set_bit={} is not a valid column and row",
                    index,
                    set_bit
                );
                None
            }
        } else {
            // No keys are pressed
            None
        }
    }

    fn bit_to_col_and_row(byte_index: u8, bit_index: u8) -> Option<(KeyColumn, KeyRow)> {
        if let Some(col) = KeyColumn::from_bit_index(bit_index % 4) {
            if let Some(row) = KeyRow::from_byte_and_bit_index(byte_index, bit_index) {
                return Some((col, row));
            }
        }

        // This is a bug in the caller, it should never happen that we encounter a combination of
        // byte and bit index that is not valid.  However it's up to the caller to handle that bug.
        None
    }
}

/// Allows to iterate over all pressed keys one at a time.  As keys are yielded from the iterator,
/// they are cleared from the struct
impl Iterator for Keys {
    type Item = (KeyColumn, KeyRow);

    fn next(&mut self) -> Option<Self::Item> {
        self.pop_key()
    }
}

impl AsRef<[u8; KEY_BYTES]> for Keys {
    fn as_ref(&self) -> &[u8; KEY_BYTES] {
        &self.0
    }
}

impl AsRef<[u8]> for Keys {
    fn as_ref(&self) -> &[u8] {
        &self.0[..]
    }
}

/// The columns in the keyboard matrix that the TM1638 scans
#[derive(Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum KeyColumn {
    K1,
    K2,
    K3,
}

impl KeyColumn {
    pub fn to_column_number(self) -> u8 {
        match self {
            Self::K1 => 1,
            Self::K2 => 2,
            Self::K3 => 3,
        }
    }

    /// Given the nibble that corresponds to some row, read the bit that corresponds to this column
    fn extract_value_from_row_nibble(&self, nibble: u8) -> bool {
        nibble & self.nibble_mask() != 0
    }

    fn nibble_mask(&self) -> u8 {
        // The representation of column bits in each nibble is a bit...odd.
        // Column K3 is in bit 0, K2 in bit 1, K1 in bit 2, and bit 3 is unused
        match self {
            Self::K1 => 0b0100,
            Self::K2 => 0b0010,
            Self::K3 => 0b0001,
        }
    }

    /// Construct a column from a (1-based) column number
    fn from_column_number(col: u8) -> Option<Self> {
        match col {
            1 => Some(Self::K1),
            2 => Some(Self::K2),
            3 => Some(Self::K3),
            _ => None,
        }
    }

    /// Construct a column from a (0-based) bit index indicating which bit in a 4-bit nibble is
    /// set.
    ///
    /// See [`Self::nibble_mask`] for an explanation of the seemingly arbitrary mapping here
    fn from_bit_index(bit: u8) -> Option<Self> {
        // `bit` must be referring to bits within a 4-bit nibble
        defmt::debug_assert!(bit < 4);
        match bit {
            0 => Some(Self::K3),
            1 => Some(Self::K2),
            2 => Some(Self::K3),
            _ => None,
        }
    }
}

/// The rows in the keyboard matrix that the TM1638 scans
#[derive(Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum KeyRow {
    KS1,
    KS2,
    KS3,
    KS4,
    KS5,
    KS6,
    KS7,
    KS8,
}

impl KeyRow {
    pub fn to_row_number(self) -> u8 {
        match self {
            Self::KS1 => 1,
            Self::KS2 => 2,
            Self::KS3 => 3,
            Self::KS4 => 4,
            Self::KS5 => 5,
            Self::KS6 => 6,
            Self::KS7 => 7,
            Self::KS8 => 8,
        }
    }

    /// Given the key bytes, extract the 4 bit nibble that corresponds to this row
    fn extract_row_nibble_from_bytes(&self, keys: &[u8; KEY_BYTES]) -> u8 {
        // Section 8 in the datasheet has a table showing which nibbles of which bytes correspond
        // to which rows
        let row: u8 = self.to_row_number();

        // Each byte contains two rows' of nibbles
        // But byte indexes are 0 based while row numbers are 1-based
        let byte = keys[((row - 1) / 2) as usize];

        // Even-numbered rows (1-based) are in the high nibble; odd numbered rows are in the low
        // nibble
        if row % 2 == 0 {
            // Even numbered row
            byte >> 4
        } else {
            // Odd numbered row
            byte & 0x0f
        }
    }

    /// Construct a row from a (1-based) row number
    fn from_row_number(col: u8) -> Option<Self> {
        match col {
            1 => Some(Self::KS1),
            2 => Some(Self::KS2),
            3 => Some(Self::KS3),
            4 => Some(Self::KS4),
            5 => Some(Self::KS5),
            6 => Some(Self::KS6),
            7 => Some(Self::KS7),
            8 => Some(Self::KS8),
            _ => None,
        }
    }

    /// Construct a row from a 0-based byte index and a 0-based bit index.
    ///
    /// This determines what row contains the key whose bit is located as the given 0-based byte
    /// and bit index.
    ///
    /// The section 8 (VIII) in the TM1638 data sheet has a table which makes this clear.
    fn from_byte_and_bit_index(byte: u8, bit: u8) -> Option<Self> {
        match byte {
            0 => {
                if bit < 4 {
                    Some(Self::KS1)
                } else {
                    Some(Self::KS2)
                }
            }
            1 => {
                if bit < 4 {
                    Some(Self::KS3)
                } else {
                    Some(Self::KS4)
                }
            }
            2 => {
                if bit < 4 {
                    Some(Self::KS5)
                } else {
                    Some(Self::KS6)
                }
            }
            3 => {
                if bit < 4 {
                    Some(Self::KS7)
                } else {
                    Some(Self::KS8)
                }
            }
            _ => None,
        }
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
                const KEY_BYTES: NonZeroU8 = unsafe { NonZeroU8::new_unchecked(4) };

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