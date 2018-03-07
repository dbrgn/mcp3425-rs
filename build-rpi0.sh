#!/bin/bash
echo "Building for Raspberry Pi Zero W..."
cargo build --release --example raspberrypi --target=arm-unknown-linux-gnueabihf
