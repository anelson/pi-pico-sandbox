/// The number of bytes used to represent the state of the keys on the board
pub const KEY_BYTES: usize = 4;

/// The state of the keys on the TM1638 board in response to a call to [`crate::Tm1638::read_keys`]
///
/// According to the datasheet, the controller supports a keypad arraged in a 3x8 matrix.  This
/// struct makes it more egonomic to work with that matrix.
///
/// Once you have read the keys from the chip and have an instance of this struct (for how to do
/// that see [`crate::Tm1638::read_keys`]), you can get the pressed key information in a few ways:
///
/// - You can use [`Self::is_pressed`] to test if a specific key was pressed when this struct was
/// read.  That might be useful if you're looking out for specific keys.
/// - You can use [`Self::any_pressed`] if you just want to know if any key was pressed without
/// knowing more.  That might be useful for waking up from sleep and turning on a display, for
/// example.
/// - The struct implements [`Iterator`] therefore it can be used in a `for` loop to
/// iterate over all pressed keys in the struct.  This is probably the most generally useful.
#[derive(Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Keys([u8; KEY_BYTES]);

impl Keys {
    pub(crate) fn new(bytes: [u8; KEY_BYTES]) -> Self {
        Self(bytes)
    }

    /// Quickly check if *any* keys are pressed
    pub fn any_pressed(&self) -> bool {
        self.0.iter().any(|byte| *byte != 0)
    }

    /// Test if the given key (identified by its column and row) is pressed
    pub fn is_pressed(&self, col: KeyColumn, row: KeyRow) -> bool {
        let nibble = row.extract_row_nibble_from_bytes(&self.0);
        #[cfg(feature = "defmt")]
        defmt::debug!("nibble={=u8}", nibble);

        col.extract_value_from_row_nibble(nibble)
    }

    /// Compress the key press data into a single 8-bit mask with one bit set if any key in the
    /// corresponding row is pressed.
    ///
    /// The TM1638 chip scans a 3x8 matrix, however the eval boards I've seen based on this chip
    /// mostly have only 8 switches.  They are wired in a way that is initially rather unintuitive,
    /// until you realize that each of the 8 switches is wired to a different row, and thus if you
    /// represent the state as an 8-bit mask of rows, you'll have the state of those switches.
    ///
    /// This is specific to that particular board, however it seems like a common pattern.  If you
    /// only need 8 or fewer switches, it makes sense to wire them one per row to make it easier to
    /// work with.
    ///
    /// This maps key rows to bits in the most intuitive way, that is: [`KeyRow::KS1`] maps to bit
    /// 0, [`KeyRow::KS2`] to bit 1, etc.
    pub fn rows_bitmask(&self) -> u8 {
        use strum::VariantArray;
        let mut bitmask = 0u8;
        for row in KeyRow::VARIANTS {
            let bit = row.to_row_number() - 1;
            let nibble = row.extract_row_nibble_from_bytes(&self.0);
            if nibble != 0 {
                // Some key in this row is set; we don't know which one and dont' care
                bitmask |= 1 << bit;
            }
        }

        bitmask
    }

    /// Helper specifically for the Handsontec MDU1093 board, which includes the TD1638 controller,
    /// 8 7-segment displays, 8 LEDs, and 8 pushbuttons switches.
    ///
    /// According to the datasheet for that board (included in the code repo), the switches are
    /// connected to the key inputs in a somewhat confusing way:
    ///
    /// S1 -> SEG1/KS1
    /// S2 -> SEG3/KS3
    /// S3 -> SEG5/KS5
    /// S4 -> SEG7/KS7
    /// S5 -> SEG2/KS2
    /// S6 -> SEG4/KS4
    /// S7 -> SEG6/KS6
    /// S8 -> SEG8/KS8
    ///
    /// The datasheet doesn't provide a hint as to why that is, but it makes it a bit inconvenient
    /// to write code for the board that responds to the switches.
    ///
    /// This helper function wraps [`rows_bitmask`] and re-arranges the results, so that bit 0
    /// corresponds to S1, bit 1 is S2, etc.
    ///
    /// This has the nice property that if you take this return value and use it to set or clear
    /// the LEDs based on the bits, you'll light up or turn off the LED that corresponds to each
    /// switch.
    #[allow(clippy::identity_op)] // The shifts by 0 bits are no-ops but I keep them in here to make the code more clear
    pub fn mdu1093_rows_bitmask(&self) -> u8 {
        // On the board the switches are mounted in a line with labels like this:
        //
        // S1  S2  S3  S4  S5  S6  S7  S8
        //
        // but because of the way they are wired, they map the rows like this:
        //
        // KS1 KS3 KS5 KS7 KS2 KS4 KS6 KS8
        //
        // Note that all 4 odd-numbered KS ports are mapped to the first four switches, followed by
        // all four even numbered KS ports to the last four switches.
        let bitmask = self.rows_bitmask();

        let s1 = bitmask & 0b0000_0001; // KS1 -> S1
        let s5 = bitmask & 0b0000_0010; // KS2 -> S5
        let s2 = bitmask & 0b0000_0100; // KS3 -> S2
        let s6 = bitmask & 0b0000_1000; // KS4 -> S6
        let s3 = bitmask & 0b0001_0000; // KS5 -> S3
        let s7 = bitmask & 0b0010_0000; // KS6 -> S7
        let s4 = bitmask & 0b0100_0000; // KS7 -> S4
        let s8 = bitmask & 0b1000_0000; // KS8 -> S8

        (s1 >> 0)
            | (s2 >> 1)
            | (s3 >> 2)
            | (s4 >> 3)
            | (s5 << 3)
            | (s6 << 2)
            | (s7 << 1)
            | (s8 << 0)
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
                #[cfg(feature = "defmt")]
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
#[derive(Copy, Clone, Debug, PartialEq, Eq, strum::VariantArray)]
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

    /// Construct a column from a (0-based) bit index indicating which bit in a 4-bit nibble is
    /// set.
    ///
    /// See [`Self::nibble_mask`] for an explanation of the seemingly arbitrary mapping here
    fn from_bit_index(bit: u8) -> Option<Self> {
        // `bit` must be referring to bits within a 4-bit nibble
        #[cfg(feature = "defmt")]
        defmt::debug_assert!(bit < 4);
        match bit {
            0 => Some(Self::K3),
            1 => Some(Self::K2),
            2 => Some(Self::K1),
            _ => None,
        }
    }
}

/// The rows in the keyboard matrix that the TM1638 scans
#[derive(Copy, Clone, Debug, PartialEq, Eq, strum::VariantArray)]
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
        let (byte_index, bit_index) = self.to_byte_and_bit_index();

        // Each byte contains two rows' of nibbles
        // But byte indexes are 0 based while row numbers are 1-based
        let byte = keys[byte_index as usize];

        (byte >> bit_index) & 0x0f
    }

    /// Get the 0-based byte index and 0-based bit index where the 4-bit nibble containing this
    /// row's column values is located.
    ///
    /// See section 8 (VIII) of the data sheet for the source of this logic.
    fn to_byte_and_bit_index(&self) -> (u8, u8) {
        // Section 8 in the datasheet has a table showing which nibbles of which bytes correspond
        // to which rows
        let row: u8 = self.to_row_number();

        let byte_index = (row - 1) / 2;

        // Even-numbered rows (1-based) are in the high nibble; odd numbered rows are in the low
        // nibble
        let bit_index = if row % 2 == 0 {
            // Even numbered row
            4
        } else {
            // Odd numbered row
            0
        };

        (byte_index, bit_index)
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

#[cfg(test)]
mod tests {
    use super::*;
    use strum::VariantArray;

    #[test]
    fn key_columns_round_trip() {
        // Test conversion of key columns to and from bit indexes
        for col in KeyColumn::VARIANTS {
            let col_number = col.to_column_number();
            let (expected_col_number, bit_index) = match *col {
                KeyColumn::K1 => (1, 2),
                KeyColumn::K2 => (2, 1),
                KeyColumn::K3 => (3, 0),
            };

            assert_eq!(col_number, expected_col_number);
            let col2 = KeyColumn::from_bit_index(bit_index).unwrap();

            assert_eq!(*col, col2, "Column converted to column number {col_number} and from there to bit index {bit_index}");

            // If we test the nibble mask for this column it should return true by definition
            assert!(col.extract_value_from_row_nibble(col.nibble_mask()));
        }
    }

    #[test]
    fn key_rows_round_trip() {
        // Test conversion of key rows to and from other forms
        for row in KeyRow::VARIANTS {
            let row_number = row.to_row_number();
            let (expected_row_number, byte_index, bit_index) = match *row {
                KeyRow::KS1 => (1, 0, 0),
                KeyRow::KS2 => (2, 0, 4),
                KeyRow::KS3 => (3, 1, 0),
                KeyRow::KS4 => (4, 1, 4),
                KeyRow::KS5 => (5, 2, 0),
                KeyRow::KS6 => (6, 2, 4),
                KeyRow::KS7 => (7, 3, 0),
                KeyRow::KS8 => (8, 3, 4),
            };

            assert_eq!(row_number, expected_row_number);
            let row2 = KeyRow::from_byte_and_bit_index(byte_index, bit_index).unwrap();

            assert_eq!(*row, row2, "Row converted to row number {row_number} and from there to byte/bit index {byte_index}/{bit_index}");
        }
    }

    #[test]
    fn no_keys_pressed() {
        // Test certain behaviors when there are no keys pressed
        let keys = Keys::new([0u8; KEY_BYTES]);
        assert_eq!(0, keys.clone().count());

        for row in KeyRow::VARIANTS {
            for col in KeyColumn::VARIANTS {
                assert!(!keys.is_pressed(*col, *row));
            }
        }

        assert_eq!(0x00, keys.rows_bitmask());
    }

    #[test]
    fn all_keys_pressed() {
        // With all keys pressed, test certain behaviors
        let keys = Keys::new([0b0111_0111u8; KEY_BYTES]);
        assert_eq!(3 * 8, keys.clone().count());

        for row in KeyRow::VARIANTS {
            for col in KeyColumn::VARIANTS {
                assert!(keys.is_pressed(*col, *row));
            }
        }

        assert_eq!(0xff, keys.rows_bitmask());
    }

    #[test]
    fn iter_invalid_keys_pressed() {
        // Two bits in each byte are unused by the controller according to the datasheet and should
        // never be set.  However anything is possible in embedded systems, maybe line noise pulls
        // the line high, or maybe the chip glitches.  The library should gracefully ignore those
        // set bits
        let keys = Keys::new([0b1000_1000u8; KEY_BYTES]);
        assert_eq!(0, keys.count());
    }

    #[test]
    fn each_row_key_pressed() {
        // For each bit set in the input, verify we translate it to the right bit corresponding to
        // the switches on the MDU1093 board, as well as the standard mapping of KS to bit.
        struct TestCase {
            key_row: KeyRow,
            std_bit_number: u8,
            mdu1093_bit_number: u8,
        }

        const TEST_CASES: &[TestCase] = &[
            TestCase {
                key_row: KeyRow::KS1,
                std_bit_number: 0,
                // S1
                mdu1093_bit_number: 0,
            },
            TestCase {
                key_row: KeyRow::KS2,
                std_bit_number: 1,
                // S5
                mdu1093_bit_number: 4,
            },
            TestCase {
                key_row: KeyRow::KS3,
                std_bit_number: 2,
                // S2
                mdu1093_bit_number: 1,
            },
            TestCase {
                key_row: KeyRow::KS4,
                std_bit_number: 3,
                // S6
                mdu1093_bit_number: 5,
            },
            TestCase {
                key_row: KeyRow::KS5,
                std_bit_number: 4,
                // S3
                mdu1093_bit_number: 2,
            },
            TestCase {
                key_row: KeyRow::KS6,
                std_bit_number: 5,
                // S7
                mdu1093_bit_number: 6,
            },
            TestCase {
                key_row: KeyRow::KS7,
                std_bit_number: 6,
                // S4
                mdu1093_bit_number: 3,
            },
            TestCase {
                key_row: KeyRow::KS8,
                std_bit_number: 7,
                // S8
                mdu1093_bit_number: 7,
            },
        ];

        for tc in TEST_CASES {
            let TestCase {
                key_row,
                std_bit_number,
                mdu1093_bit_number,
            } = tc;

            // Make a keys array with column 1 of this key row set
            let mut keys = [0u8; KEY_BYTES];
            let (byte_index, bit_index) = key_row.to_byte_and_bit_index();

            // There are three bits for the three columns in each row.  We can set them all, it
            // doesn't matter; only one column has to be set for each row in order for
            // `rows_bitmask` to set the bit for that row.
            keys[byte_index as usize] = 0b0000_0001 << bit_index;

            let keys = Keys::new(keys);

            // This should report only one key pressed; Column 3 of the test row
            // Why column 3?  Because, confusingly, the least significant bit of the row's nibble
            // corresponds to K3, followed by K2 in the next bit, and K1 after that.
            //
            // On the MDU1093 board, all of the switches are wired into column K3
            assert_eq!(Some((KeyColumn::K3, *key_row)), keys.clone().next());
            assert_eq!(1, keys.clone().count());

            let rows_bitmask = keys.rows_bitmask();

            assert_eq!(rows_bitmask, (1 << std_bit_number),
                    "Rows bitmask {rows_bitmask:b} doesn't reflect pressed keys in row {key_row:?}; expected result to appear in bit {std_bit_number}");

            let rows_bitmask = keys.mdu1093_rows_bitmask();

            assert_eq!(rows_bitmask, (1 << mdu1093_bit_number),
                    "MDU1093 Rows bitmask {rows_bitmask:b} doesn't reflect pressed keys in row {key_row:?}; expected result to appear in bit {mdu1093_bit_number}");
        }
    }
}
