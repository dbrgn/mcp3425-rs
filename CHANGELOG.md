# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/en/1.0.0/)
and this project adheres to [Semantic Versioning](http://semver.org/spec/v2.0.0.html).


## [1.0.0] - 2022-12-24

This release adds support for the MCP3426/7/8 models, at the cost of a slightly
adjusted API. Thanks @marius-meissner for contributing and testing this
feature!

### Added

- Support for multi-channel measurements (#10)
- Support for MCP3426/7/8 models (#10)

### Changed

- The `Config::new(...)` constructor function is now gone. Use
  `Config::default()` with the builder methods to create your measurement
  config instead.
- Switch to Rust 2021 edition (#14)
- Upgrade linux-embedded-hal dependency: 0.2 → 0.3
- Upgrade measurements dependency: 0.10 → 0.11


## [0.3.0] - 2018-05-15

### Changed

- Upgrade to embedded-hal 0.2
- Upgrade to measurements 0.10

### Fixed

- Fix continuous example: The "not ready" status is now an error,
  which breaks the example.


## [0.2.1] - 2018-03-13

This is only a docs update.


## [0.2.0] - 2018-03-13

### Changed

- Remove `Measurement` type, `Measurement::NotFresh` is now `Error::NotReady`
- Shorter delay times (shorter blocking)

### Added

- Add `Voltage` wrapper type (#6)
- Add conversion functions between measurement modes (#1)
- Add optional integration with `measurements` crate (#7)


## [0.1.1] - 2018-03-11

### Changed

- In `MCP3425::set_config`, block until the first measurement is ready


## 0.1.0 - 2018-03-11

This is the initial release to crates.io of the feature-complete driver. There
may be some API changes in the future, in case I decide that something can be
further improved. All changes will be documented in this CHANGELOG.


[1.0.0]: https://github.com/dbrgn/mcp3425-rs/compare/v0.3.0...v1.0.0
[0.3.0]: https://github.com/dbrgn/mcp3425-rs/compare/v0.2.1...v0.3.0
[0.2.1]: https://github.com/dbrgn/mcp3425-rs/compare/v0.2.0...v0.2.1
[0.2.0]: https://github.com/dbrgn/mcp3425-rs/compare/v0.1.1...v0.2.0
[0.1.1]: https://github.com/dbrgn/mcp3425-rs/compare/v0.1.0...v0.1.1
