#!/bin/bash
echo "Building examples for Raspberry Pi Zero W..."
echo ""
echo "=> oneshot"
cargo build --release --example oneshot --target=arm-unknown-linux-gnueabihf
echo ""
echo "=> continuous"
cargo build --release --example continuous --target=arm-unknown-linux-gnueabihf
