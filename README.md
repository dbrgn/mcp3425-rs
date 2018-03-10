# Rust MCP3425 Driver

[![Crates.io Version][crates-io-badge]][crates-io]
[![Crates.io Downloads][crates-io-download-badge]][crates-io-download]

This is a platform agnostic Rust driver for the MCP3425, based on the
[`embedded-hal`](https://github.com/japaric/embedded-hal) traits.


## The Device

The Microchip MCP3425 is a low-current 16-bit analog-to-digital converter.

The device has an I²C interface and an on-board ±2048mV reference.

Details and datasheet: http://www.microchip.com/wwwproducts/en/en533561


## Status

- [x] Support one-shot measurements
- [x] Support continuous measurements
- [x] Configurable sample rate / resolution
- [x] Configurable gain (PGA)
- [x] Handle saturation values (high and low)
- [x] Docs
- [ ] Polish & release to crates.io


## License

Licensed under either of

 * Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
   http://www.apache.org/licenses/LICENSE-2.0)
 * MIT license ([LICENSE-MIT](LICENSE-MIT) or
   http://opensource.org/licenses/MIT) at your option.


### Contributing

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall
be dual licensed as above, without any additional terms or conditions.


<!-- Badges -->
[crates-io]: https://crates.io/crates/mcp3425
[crates-io-badge]: https://img.shields.io/crates/v/mcp3425.svg?maxAge=3600
[crates-io-download]: https://crates.io/crates/mcp3425
[crates-io-download-badge]: https://img.shields.io/crates/d/mcp3425.svg?maxAge=3600
