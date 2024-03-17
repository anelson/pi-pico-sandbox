//! Module describing the [`BusDriver`] trait and various implementations depending on target
//! platform.

// This module defines traits w/ async methods.  That triggers a warning due to the very...limited
// support for this in the current Rust version (1.76 as of this writing).  However this pertains
// only to the use of futures returned by async methods in multi-threaded executors.  As this crate
// is meant for use on embedded microcontrollers without any concept of threads, this does not
// concern us at all
#![allow(async_fn_in_trait)]

use core::marker::PhantomData;

/// This trait represents some low-level implementation of the TM1638 bus interface, likely in
/// terms of some platform-specific HAL.
///
/// The TM1638 uses a three-wire bus similar to SPI, but not so similar that we can just use an SPI
/// implementation instead.  This trait exposes a byte-level interface that must be implemented by
/// a bus driver in terms of bit-level I/O, either using bit-banging, PIO, or maybe some hacked
/// version of an SPI implementation.
///
/// Depending on the features enabled in this crate, there are some built-in implementations
/// available for various HALs.
///
/// Sadly, due to [this issue](https://github.com/rust-embedded/embedded-hal/issues/397), it's not
/// currently possible to write a generic implementation purely in terms of Embedded HAL traits.
/// If that issue is ever resolved and a suitable abstraction for input/output pins devised, this
/// trait can go away entirely.
pub trait BusDriver {
    type Error;

    /// Send a single command, with no payload, and no response expected
    async fn send_command(&mut self, b: u8) -> Result<(), Self::Error>;

    /// Send a command with a data payload, but no response expected.
    async fn send_command_write_data(&mut self, b: u8, data: &[u8]) -> Result<(), Self::Error>;

    /// Send a command which is expected to generate a response.
    ///
    /// The expected size of the response (in bytes) is determined by the size of the `data` slice.
    /// This operation will return once enough bytes are received to fill `data`.
    async fn send_command_read_data(&mut self, b: u8, data: &mut [u8]) -> Result<(), Self::Error>;
}

/// The bit-banging bus driver uses this trait to abstract away the very low-level details of
/// controlling the three pins connected to the TM1638 device.  Other drivers might or might not
/// need this.
pub trait Pins {
    type Error;

    fn set_clk(&mut self, state: bool) -> Result<(), Self::Error>;

    fn set_stb(&mut self, state: bool) -> Result<(), Self::Error>;

    /// Put the DIO pin into output mode (followed by calls to [`Self::set_dio`]
    fn set_dio_as_output(&mut self) -> Result<(), Self::Error>;
    fn set_dio(&mut self, state: bool) -> Result<(), Self::Error>;

    /// Put the DIO pin into input mode (followed by calls to [`Self::get_dio`])
    fn set_dio_as_input(&mut self) -> Result<(), Self::Error>;

    /// Read the state of the DIO pin, returning `true` for high and `false` for low.
    fn get_dio(&mut self) -> Result<bool, Self::Error>;
}

/// Abstraction on platform-specific timers to provide a generic way to pause the bus driver
/// execution in order to implement the TM1638 bus protocol correctly.
///
/// The timer situation on embedded Rust is still quite unstable, with competing timer
/// implementations, including `embasssy_time`, `embedded-time`, `fugit`, and probably others.  To
/// avoid picking a side, this very simple timer trait needs to be implemented in terms of whatever
/// your preferred timer implementation is.
pub trait Timer {
    /// Wait for the clock interval to ensure an outgoing value on the DIO pin is read.
    /// This should be at least 1us.  It can be more, but obviously that impacts the speed with
    /// which we can communicate with the board.
    async fn wait_clock_tick();

    /// Wait for the tWAIT interval defined in section 12 of the datasheet, Timing Characteristics.
    /// This is also at least 1us, but it's kept as a separate function out of an abundance of
    /// caution in case there emerges a reason to use different intervals for each.  By default it
    /// is implemented in terms of `wait_clock_tick`
    async fn wait_twait() {
        Self::wait_clock_tick().await
    }
}

/// Implementation of [`BusDriver`] by bit-banging GPIO pins.
///
/// The actual control over the GPIO pins is abstracted away behind yet another trait,
/// [`Pins`].  Thus this bit-banging implementation is available on any target and with
/// any HAL for which a `Pins` implementation exists.
pub struct BitBangingBusDriver<P: Pins, T: Timer> {
    pins: P,
    _timer: PhantomData<T>,
}

impl<P: Pins, T: Timer> BitBangingBusDriver<P, T> {
    pub fn new(mut pins: P) -> Result<Self, P::Error> {
        // Initially the DIO pin is for output, except when scanning for key presses
        pins.set_dio_as_output()?;

        // Set the pins in their initial conditions, corresponding to an idle state
        pins.set_dio(false)?;

        Ok(Self {
            pins,
            _timer: Default::default(),
        })
    }

    /// Send a single byte, maybe command maybe data this low level function doesn't care.
    ///
    /// Sends a bit at a time on the DIO pin
    async fn send_byte(&mut self, b: u8) -> Result<(), P::Error> {
        self.shift_byte_out(b).await
    }

    /// Shift the byte value out on the DIO pin, LSB first, waiting an appropriate
    /// period of time between clock pulses to ensure the TM1638 can keep up
    ///
    /// Assumes the DIO pin has already been set up as output
    async fn shift_byte_out(&mut self, b: u8) -> Result<(), P::Error> {
        for bit in 0..8 {
            let mask = 1 << bit;
            let value = (b & mask) != 0;

            self.pins.set_dio(value)?;

            // XXX Experiment: the LA sometimes shows the clock going high within a few nano of the
            // DIO pin going high.  It's not clear if the chip will read this as a 0 or a 1.  Try
            // waiting a tick for the DIO pin to assume its state before bringing clock high.  The
            // datasheet says data is read from DIO on the rising edge of CLK so this detail
            // matters.
            T::wait_clock_tick().await;

            self.pins.set_clk(true);
            T::wait_clock_tick().await;
            self.pins.set_clk(false);
            T::wait_clock_tick().await;
        }

        Ok(())
    }

    /// Shift a byte value in from the DIO pin, LSB first, using the CLK pin to drive the
    /// controller to send data.
    ///
    /// Assumes the DIO pin is already set up as input
    async fn shift_byte_in(&mut self) -> Result<u8, P::Error> {
        let mut value = 0;

        for bit in 0..8 {
            // Strobe clock HIGH to signal controller to read a bit
            self.pins.set_clk(true)?;
            T::wait_clock_tick().await;

            let mask = 1 << bit;

            if self.pins.get_dio()? {
                value |= mask;
            }

            self.pins.set_clk(false)?;
            T::wait_clock_tick().await;
        }

        Ok(value)
    }
}

impl<P: Pins, T: Timer> BusDriver for BitBangingBusDriver<P, T> {
    type Error = P::Error;

    /// Send a single byte that represents a command, so strobe will be pulled low
    /// before the command's bits are sent, and then pulled high again after.
    async fn send_command(&mut self, b: u8) -> Result<(), Self::Error> {
        self.pins.set_stb(false)?;
        self.send_byte(b).await?;
        self.pins.set_stb(true)?;

        Ok(())
    }

    /// Send a single byte that represents a command followed by one or more data bytes, so strobe
    /// will be pulled low before the command's bits are sent, and not pulled high again
    /// until after the data bytes are sent.
    async fn send_command_write_data(&mut self, b: u8, data: &[u8]) -> Result<(), Self::Error> {
        #[cfg(feature = "defmt")]
        defmt::debug_assert!(!data.is_empty());
        self.pins.set_stb(false)?;
        self.send_byte(b).await;
        for b in data {
            #[cfg(feature = "defmt")]
            defmt::trace!("data byte = {=u8:x}", b);
            self.send_byte(*b).await;
        }
        self.pins.set_stb(true)?;

        Ok(())
    }

    /// Send a single byte that represents a command and which expects a response back from the
    /// controller, so strobe will be pulled low before the command's bits are sent, and then pulled high again after all bytes are read.
    async fn send_command_read_data(&mut self, b: u8, data: &mut [u8]) -> Result<(), Self::Error> {
        self.pins.set_stb(false)?;
        self.send_byte(b).await?;

        // We will be reading from DIO
        self.pins.set_dio_as_input()?;

        // Wait Twait interval before reading response
        T::wait_twait().await;

        #[cfg(feature = "defmt")]
        defmt::trace!("Expecting {0} bytes from controller", data.len());

        for byte in data.iter_mut() {
            *byte = self.shift_byte_in().await?;
        }

        // Done reading from DIO, put it back to output
        self.pins.set_dio_as_output()?;

        self.pins.set_stb(true)?;

        Ok(())
    }
}

#[cfg(feature = "embassy-time")]
mod embassy_time_timer {
    use embassy_time::{Duration, Timer as EmbassyTimer};

    /// Use a 1uS clock tick to ensure the TM1638 picks up the value
    const CLOCK_TICK: Duration = Duration::from_micros(1);

    /// The interval to wait after sending the button read command, before reading data
    /// Corresponds to tWAIT in section 12 of the datasheet, under Timing Characteristics.
    const TWAIT: Duration = Duration::from_micros(1);

    pub struct EmbassyTimeTimer;

    impl super::Timer for EmbassyTimeTimer {
        async fn wait_clock_tick() {
            EmbassyTimer::after(CLOCK_TICK).await
        }

        async fn wait_twait() {
            EmbassyTimer::after(TWAIT).await
        }
    }
}

#[cfg(feature = "embassy-time")]
pub use embassy_time_timer::*;

#[cfg(feature = "embassy-rp")]
mod embassy_rp_bus_driver {
    use core::convert::Infallible;
    use embassy_rp::gpio;

    /// Implementation of [`super::Pins`] that uses the Embassy RP HAL for the RP2040
    /// microcontroller.
    pub struct EmbassyRpPins<'a, StrobePin: gpio::Pin, ClockPin: gpio::Pin, DioPin: gpio::Pin> {
        strobe: gpio::Output<'a, StrobePin>,
        clock: gpio::Output<'a, ClockPin>,
        dio: gpio::Flex<'a, DioPin>,
    }

    /// Convenient type alias for the bit-banging bus driver using the `embassy-rp` HAL for RP2040.
    pub type EmbassyRpBusDriver<'a, StrobePin, ClockPin, DioPin, T> =
        super::BitBangingBusDriver<EmbassyRpPins<'a, StrobePin, ClockPin, DioPin>, T>;

    impl<'a, StrobePin: gpio::Pin, ClockPin: gpio::Pin, DioPin: gpio::Pin>
        EmbassyRpPins<'a, StrobePin, ClockPin, DioPin>
    {
        pub fn new(strobe: StrobePin, clock: ClockPin, dio: DioPin) -> Self {
            Self {
                strobe: gpio::Output::new(strobe, gpio::Level::High),
                clock: gpio::Output::new(clock, gpio::Level::Low),
                dio: gpio::Flex::new(dio),
            }
        }
    }

    impl<'a, StrobePin: gpio::Pin, ClockPin: gpio::Pin, DioPin: gpio::Pin> super::Pins
        for EmbassyRpPins<'a, StrobePin, ClockPin, DioPin>
    {
        type Error = Infallible;

        fn set_clk(&mut self, state: bool) -> Result<(), Self::Error> {
            self.clock.set_level(state.into());
            Ok(())
        }

        fn set_stb(&mut self, state: bool) -> Result<(), Self::Error> {
            self.strobe.set_level(state.into());
            Ok(())
        }

        fn set_dio_as_output(&mut self) -> Result<(), Self::Error> {
            self.dio.set_as_output();

            Ok(())
        }

        fn set_dio(&mut self, state: bool) -> Result<(), Self::Error> {
            self.dio.set_level(state.into());

            Ok(())
        }

        fn set_dio_as_input(&mut self) -> Result<(), Self::Error> {
            self.dio.set_as_input();

            Ok(())
        }

        fn get_dio(&mut self) -> Result<bool, Self::Error> {
            Ok(self.dio.get_level().into())
        }
    }
}

#[cfg(feature = "embassy-rp")]
pub use embassy_rp_bus_driver::*;
