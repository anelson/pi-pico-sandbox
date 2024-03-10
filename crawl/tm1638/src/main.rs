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

    info!("Hello World!");

    tm1638::foo().await;
}
