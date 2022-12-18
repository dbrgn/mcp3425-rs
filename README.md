# Rust MCP3425/6/7/8 Driver

[![GitHub Actions][github-actions-badge]][github-actions]
[![Crates.io Version][crates-io-badge]][crates-io]
[![Crates.io Downloads][crates-io-download-badge]][crates-io-download]

This is a platform agnostic Rust driver for the MCP3425 (and newer variants
MCP3426/MCP3427/MCP3428 as well), based on the
[`embedded-hal`](https://github.com/rust-embedded/embedded-hal) traits.

Docs: https://docs.rs/mcp3425

Introductory blogpost: https://blog.dbrgn.ch/2018/3/13/rust-mcp3425-driver/


## The Device

The Microchip MCP3425 is a low-current 16-bit analog-to-digital converter.

The device has an I²C interface and an on-board ±2048mV reference.

Details and datasheet: https://ww1.microchip.com/downloads/aemDocuments/documents/OTH/ProductDocuments/DataSheets/22072b.pdf

Variants [MCP3426/7/8](https://ww1.microchip.com/downloads/en/DeviceDoc/22226a.pdf)
are very similar, but support multiple input channels. They are supported as
well, but require to enable one of the following Cargo features:

- `dual_channel` for MCP3426/7
- `quad_channel` for MCP3428


## Status

- [x] Support one-shot measurements
- [x] Support continuous measurements
- [x] Configurable sample rate / resolution
- [x] Configurable gain (PGA)
- [x] Configurable channel (only MCP3426/7/8)
- [x] Handle saturation values (high and low)
- [x] Docs


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
[github-actions]: https://github.com/dbrgn/mcp3425-rs/actions/workflows/ci.yml
[github-actions-badge]: https://github.com/dbrgn/mcp3425-rs/actions/workflows/ci.yml/badge.svg
[crates-io]: https://crates.io/crates/mcp3425
[crates-io-badge]: https://img.shields.io/crates/v/mcp3425.svg?maxAge=3600
[crates-io-download]: https://crates.io/crates/mcp3425
[crates-io-download-badge]: https://img.shields.io/crates/d/mcp3425.svg?maxAge=3600
