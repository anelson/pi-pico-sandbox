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
    let driver = tm1638::EmbassyRpBusDriver::<_, _, _, tm1638::EmbassyTimeTimer>::new(
        p.PIN_6, p.PIN_7, p.PIN_8,
    );
    let mut tm1638 = tm1638::Tm1638::new(driver);
    tm1638.init().await.unwrap();

    debug!("Hello!  Press one of the buttons on the board!");

    loop {
        tm1638.blank_display().await.unwrap();
        let keys = tm1638.read_keys().await.unwrap();

        if keys.any_pressed() {
            // Go apeshit
            tm1638.activate_display(0x07).await.unwrap();

            for (col, row) in keys.clone() {
                // Sanity check the result, to make sure the enumeration of pressed keys agrees
                // with the logic to test if a given key is pressed
                if !keys.is_pressed(col, row) {
                    error!("BUG: This column and row is not pressed!");
                }

                debug!("col {}, row {}", col, row);

                let col: u8 = col.to_column_number();
                let row: u8 = row.to_row_number();

                // Set the display corresponding to the row to a mask corresponding to the column
                // number
                tm1638.set_display_mask(row - 1, col).await.unwrap();

                // Light the LED as well
                tm1638.set_led_mask(row - 1, 0xff).await.unwrap();
            }
        }

        Timer::after_millis(10).await;
    }
}
