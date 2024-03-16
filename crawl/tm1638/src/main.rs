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

    let mut led_state = false;

    loop {
        tm1638.deactivate_display().await;

        let new_mask = if led_state {
            // Turn all segment displays and LEDs off
            led_state = false;
            0x00
        } else {
            // Turn everything on
            led_state = true;
            0xff
        };

        tm1638.activate_display(0x01).await;

        for display in 0..8 {
            tm1638.set_display_mask(display, new_mask).await;
        }

        Timer::after_millis(500).await;

        for led in 0..8 {
            tm1638.set_led_mask(led, new_mask).await;
        }

        Timer::after_millis(500).await;

        // Gradually increase the brightness
        for brightness in 2..=7 {
            tm1638.activate_display(brightness).await;
            Timer::after_millis(500).await;
        }
    }
}
