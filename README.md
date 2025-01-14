# Rubble

[![crates.io](https://img.shields.io/crates/v/rubble.svg)](https://crates.io/crates/rubble)
[![docs.rs](https://docs.rs/rubble/badge.svg)](https://docs.rs/rubble/)
[![Build Status](https://travis-ci.org/jonas-schievink/rubble.svg?branch=master)](https://travis-ci.org/jonas-schievink/rubble)

Rubble is a Bluetooth® Low Energy compatible protocol stack for embedded Rust.

Currently, Rubble supports Nordic's nRF52-series of MCUs. However, it was
designed to be hardware-independent, so support crates for other MCU families
are always welcome.

[API documentation (master)](https://jonas-schievink.github.io/rubble/)

**NOTE: None of this has passed the Bluetooth® Qualification Process, so it
must not be used in commercial products!**

## Usage

See [demos](./demos/) for usage examples.

```bash
$ cd rubble-demo
$ cp .gdbinit-openocd .gdbinit
$ cargo run
```

Logging is done over UART, TX and RX on pin 06 and 08 respectively, at 1MBd.

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md).

## License

0-Clause BSD License ([LICENSE](LICENSE)).
