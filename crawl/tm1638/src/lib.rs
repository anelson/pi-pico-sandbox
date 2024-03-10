#![no_std]

use defmt::*;
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

pub struct Tm1638<'a, StrobePin: gpio::Pin, ClockPin: gpio::Pin, DioPin: gpio::Pin> {
    strobe: gpio::Output<'a, StrobePin>,
    clock: gpio::Output<'a, ClockPin>,
    dio: gpio::Flex<'a, DioPin>,
}

impl<'a, StrobePin: gpio::Pin, ClockPin: gpio::Pin, DioPin: gpio::Pin>
    Tm1638<'a, StrobePin, ClockPin, DioPin>
{
    pub fn new(strobe: StrobePin, clock: ClockPin, dio: DioPin) -> Self {
        Self {
            strobe: gpio::Output::new(strobe, gpio::Level::High),
            clock: gpio::Output::new(clock, gpio::Level::Low),
            dio: gpio::Flex::new(dio),
        }
    }

    /// Reset the TM1638 state, blanking all of the LEDs
    pub async fn init(&mut self) {
        self.send_command_byte(TM_ACTIVATE).await;

        self.send_command_byte(TM_BRIGHT_ADR + (TM_BRIGHT_MASK & TM_DEFAULT_BRIGHTNESS))
            .await;
        self.send_command_byte(TM_WRITE_INC).await;
        self.strobe.set_low();
        self.send_byte(TM_SEG_ADR).await;
        for _ in 0..16 {
            self.send_byte(0x00).await;
        }
        self.strobe.set_high();

        // Initially the DIO pin is for output, except when scanning for key presses
        self.dio.set_as_output();
    }

    async fn apply_command<'c>(&self, command: Command<'c>) {
        match command {
            Command::WriteInc { start_segment_addr } => defmt::todo!(),
            Command::SendDataByte { data } => defmt::todo!(),
            Command::SendDataSlice { data } => defmt::todo!(),
        }
    }

    /// Send a single byte that represents a command, so strobe will be pulled low
    /// before the command's bits are sent, and then pulled high again after.
    async fn send_command_byte(&mut self, b: u8) {
        self.strobe.set_low();
        self.send_byte(b).await;
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

enum Command<'a> {
    /// Write to the display addresses, automatically incrementing the target address
    WriteInc {
        /// The initial address of the segment to write to, relative to the starting address fo the
        /// first segment.  So a value of `0` means the first segment, `1` the second segment, etc.
        ///
        /// The controller will automaticaly increment the address after every data byte written.
        ///
        /// Follow this with a [`Self::SendDataByte`] or [`Self::SendDataSlice`] command
        start_segment_addr: u8,
    },
    /// Write a single byte of data to the controller
    SendDataByte { data: u8 },
    /// Write a slice of data to the controller
    SendDataSlice { data: &'a [u8] },
}

impl<'a> Command<'a> {
    fn encode(self) -> (Option<u8>, Option<&'a [u8]>) {
        defmt::todo!()
    }
}

pub async fn foo() {
    info!("Foo!");
    //let mut led = Output::new(p.PIN_25, Level::Low);
    //let mut async_input = Input::new(p.PIN_16, Pull::None);
    //
    //loop {
    //    info!("wait_for_high. Turn on LED");
    //    led.set_high();
    //
    //    async_input.wait_for_high().await;
    //
    //    info!("done wait_for_high. Turn off LED");
    //    led.set_low();
    //
    //    Timer::after_secs(2).await;
    //}
}
