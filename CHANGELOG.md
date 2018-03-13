# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/en/1.0.0/)
and this project adheres to [Semantic Versioning](http://semver.org/spec/v2.0.0.html).

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

[0.1.1]: https://github.com/dbrgn/mcp3425/compare/v0.1.0...v0.1.1
