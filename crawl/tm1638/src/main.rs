//! Template

#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::{
    self,
    i2c::{self, Config},
};
use embassy_time::Timer;
use embedded_hal_1::i2c::I2c;
use {defmt_rtt as _, panic_probe as _};

use tm1638;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // Instantiate the TM1638 interface
    let mut tm1638 = tm1638::Tm1638::new(p.PIN_6, p.PIN_7, p.PIN_8);
    tm1638.init().await;

    debug!("here we go!");

    loop {
        tm1638.blank_display().await;
        let keys = tm1638.read_keys().await;

        if keys.any_pressed() {
            // Go apeshit
            tm1638.activate_display(0x07).await;

            for (col, row) in keys {
                let col: u8 = col.to_column_number();
                let row: u8 = row.to_row_number();

                debug!("col {=u8}, row {=u8}", col, row);

                // Set the display corresponding to the row to a mask corresponding to the column
                // number
                tm1638.set_display_mask(row - 1, col).await;

                // Light the LED as well
                tm1638.set_led_mask(row - 1, 0xff).await;
            }
        }

        Timer::after_millis(10).await;
    }
}
