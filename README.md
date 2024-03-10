# Experiments w/ Pi Pico

## Pico W on Rust

rp-rs/rp-hal#380 is a (seemingly abandoned) PR to as a BSP for Pico W to `rp-hal` which is where the HAL for the Pi Pico
lives.  I fell down this rat hole because I wanted to run a simple "blinky" example in Rust, to blink the LED, but on
the Pico W that LED is connected to the wireless controller that you have to talk to using SPI.  Apparently the C/C++
SDK abstract that away somehow so it seemed as easy as setting a GPIO pin.

By contrast the `embassy-rs` project focusing on supporting async on embedded Rust, has this working.

```
cd examples/rp && cargo run --bin wifi_blinky
```

in the `embassy` source tree.

I have my reservations about introducing the complexity of async on top of the complexity of embedded development.  But
several of the use cases I have in mind need wifi so using a HAL that doesn't actually support the use of the wireless
interface is definitely a deal-breaker.

The LED blink is a stupid, small thing, but I fear it's a sign of more widespread indifference to this platform.  If
that doesn't work, what about a TCP/IP stack?  An MQTT client?

## New Embassy Project

`cargo generate` template at https://github.com/bentwire/embassy-rp2040-template.  The docs for the Rust Pico HAL also
have a `cargo generate` template for their libraries, but I'm interested in playing w/ that async hotness so I'm taking
the hard path.


