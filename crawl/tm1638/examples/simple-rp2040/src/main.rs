//! Example of using the TM1638 on an RP2040 board like the Pi Pico

#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp;
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

use tm1638;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // Instantiate the TM1638 interface using a bit-banging implementation of the TM1638 bus
    // interface implemented using the `embassy-rp` HAL and the `embassy-time` timer.
    let mut tm1638 = tm1638::Tm1638::builder()
        .with_embassy_timer()
        .with_embassy_rp_pins(p.PIN_6, p.PIN_7, p.PIN_8)
        .build()
        .unwrap();
    tm1638.init().await.unwrap();

    debug!("Hello!  Press one of the buttons on the board!");

    loop {
        let keys = tm1638.read_keys().await.unwrap();

        if keys.any_pressed() {
            // Go apeshit
            tm1638.activate_display(0x07).await.unwrap();

            let mut bitmask = keys.mdu1093_rows_bitmask();

            for row in 0..8 {
                // Illuminate the LEDs for any rows that have a pressed key in them
                // On the TM1638 board I have, each of the 8 switches is wired into a different row
                // so this works well.
                tm1638.set_led_mask(row, bitmask & 0x01).await.unwrap();
                bitmask >>= 1;
            }

            for (col, row) in keys.clone() {
                // Sanity check the result, to make sure the enumeration of pressed keys agrees
                // with the logic to test if a given key is pressed
                if !keys.is_pressed(col, row) {
                    error!("BUG: This column and row is not pressed!");
                }

                debug!("col {}, row {}", col, row);

                //let col: u8 = col.to_column_number();
                //let row: u8 = row.to_row_number();
                //
                // Set the display corresponding to the row to a mask corresponding to the column
                // number
                //tm1638.set_display_mask(row - 1, col).await.unwrap();
            }
        }

        Timer::after_millis(10).await;
    }
}
