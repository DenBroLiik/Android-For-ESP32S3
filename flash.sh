#!/bin/bash
source /home/denbroliik/export-esp.sh

# Build the bootloader
sudo -E PATH="$PATH" cargo build --release --bin bootloader

# Check if the user wants to flash or connect to a serial port
read -p "Do you want to flash the device (f) or connect to a serial port (s)? [f/s]: " choice

if [ "$choice" = "f" ] || [ "$choice" = "F" ]; then
    # Flash the device
    # espflash flash --erase-parts=all --monitor --chip esp32s3 target/xtensa-esp32s3-none-elf/release/bootloader
    sudo -E PATH="$PATH" cargo run --release --bin bootloader
elif [[ "$choice" =~ ^[Ss]$ ]]; then
    read -p "Enter port (leave empty for auto-detect): " port
    if [ -z "$port" ]; then
        espflash monitor
    else
        espflash monitor --port "$port"
    fi
else
    echo "Invalid choice. Exiting."
fi
