# TM1638 experiment

This was created using `cargo generate` from https://github.com/bentwire/embassy-rp2040-template .  I chose "yes" to all
the options like USB and networking support.  Probably unnecessarily.

This is just an experiment.  I want to adapt the C++ code at https://github.com/gavinlyonsrepo/TM1638plus_PICO to async
Rust using Embassy, and be able to communicate with the TM1638-based board I have for experiments.


## HAL gotcha

In the [embedded-hal repo issue](https://github.com/rust-embedded/embedded-hal/issues/397) about traits for pins that
are both input and output, it's clear that this isn't handled in `embedded-hal` and a solution is not going to be
quickly forthcoming.  Bummer.

This part of the ecosystem is still very immature.  It seems I'll need my own abstraction to make this sufficiently
generalized that it will both work on the embedded platform I want, but can run on desktop computers running Rust for
testing purposes.
