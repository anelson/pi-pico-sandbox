This is something I had to hack together to get working.

It started out life as the `embassy` example `wifi_blinky.rs`, which out of the box just blinks the LED on the Pico W.
I combined it with another example `debounce` which debounces a button (not the on-board button) connected to a GPIO
pin, and turns on the on-board LED only when the button is down.

I made this work initialy by editing the `wifi_blink.rs` example in the Embassy source tree, so all of the crate
dependencies were path dependencies.  But I wanted this to work in my separate repo, so I copied the source over and
copy-pasted the same dependencies from Embassy, and then it all broke.  `embassy-rp` 0.1 is the most recent on published
on crates.io, but it's ancient.  There's been a hugely breaking refactor since then, where the pin is no longer a type
parameter to `Input` and `Output`.  Frankly it's a big improvement but the only problem is it's not released yet.  So
I could either take a git dependency like a chump, or fix the example.  I did that, so now the pin numbers are part of
the type signatures and the code is just shit.  But it works!

Going forward if I am able to use Embassy for my projects at all, I get the sense I'll probably be using git
dependencies with all of the nightmare that entails.  FML.


