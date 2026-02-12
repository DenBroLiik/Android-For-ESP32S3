#!/bin/bash
source /home/denbroliik/export-esp.sh
sudo -E PATH="$PATH" cargo run --release --bin bootloader && \
espflash flash --erase-parts=all --monitor --chip esp32s3 target/xtensa-esp32s3-none-elf/release/bootloader
