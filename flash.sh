#!/bin/bash
source ~/export-esp.sh

echo "Отчистка"
cargo clean

# ==========================================
# НОВАЯ ЛОГИКА: Проверка и подготовка vendor
# ==========================================
VENDOR_DIR="./vendor"
CONFIG_TOML="./config/config.toml"
RHAI_CARGO="./rhai/Cargo.toml"

# 1 & 2. Проверяем, есть ли ./vendor и не пуста ли она
if [ ! -d "$VENDOR_DIR" ] || [ -z "$(ls -A "$VENDOR_DIR" 2>/dev/null)" ]; then
    echo "[1-3] Директория vendor отсутствует или пуста. Запуск 'cargo vendor'..."
    # 3. Запускаем cargo vendor
    if ! cargo vendor; then
        echo "[4] Ошибка 'cargo vendor'. Комментируем настройки vendored-sources в $CONFIG_TOML..."
        # 4. Комментируем указанные строки в config.toml
        if [ -f "$CONFIG_TOML" ]; then
            sed -i 's|^\(\s*\[source\.crates-io\]\)|# \1|' "$CONFIG_TOML"
            sed -i 's|^\(\s*replace-with = "vendored-sources"\)|# \1|' "$CONFIG_TOML"
            sed -i 's|^\(\s*\[source\.vendored-sources\]\)|# \1|' "$CONFIG_TOML"
            sed -i 's|^\(\s*directory = "vendor"\)|# \1|' "$CONFIG_TOML"
        fi
    fi
fi

# 5. Правка ./rhai/Cargo.toml
if [ -f "$RHAI_CARGO" ]; then
    if grep -q '^\s*no-std-compat = { path = "../vendor/no-std-compat"' "$RHAI_CARGO"; then
        echo "[5] Временно переключаем no-std-compat на версию из crates.io..."
        # Комментируем строку с path
        sed -i 's|^\(\s*no-std-compat = { path = "../vendor/no-std-compat".*}\)|# \1|' "$RHAI_CARGO"
        # Раскомментируем строку без path
        sed -i 's|^#\s*\(no-std-compat = { version = "0.4.1", default-features = false, features = \["alloc"\], optional = true }\)|\1|' "$RHAI_CARGO"
    fi
fi

# 6. Повторный запуск cargo vendor
echo "[6] Повторный запуск 'cargo vendor'..."
if ! cargo vendor; then
    echo "❌ Критическая ошибка: повторный 'cargo vendor' завершился с ошибкой."
    exit 1
fi

# 7. Реверс 5-го действия (восстановление оригинала в rhai/Cargo.toml)
if [ -f "$RHAI_CARGO" ]; then
    echo "[7] Восстанавливаем оригинальную зависимость no-std-compat..."
    # Раскомментируем строку с path
    sed -i 's|^#\s*\(no-std-compat = { path = "../vendor/no-std-compat".*}\)|\1|' "$RHAI_CARGO"
    # Комментируем строку без path
    sed -i 's|^\(\s*no-std-compat = { version = "0.4.1", default-features = false, features = \["alloc"\], optional = true }\)|# \1|' "$RHAI_CARGO"
fi

echo "✅ Подготовка vendor завершена."
# ==========================================

# Дальше по оригинальному скрипту
# Build the bootloader
cargo build --release --bin bootloader

# Check if the user wants to flash or connect to a serial port
read -p "Do you want to flash the device (f) or connect to a serial port (s)? [f/s]: " choice
if [ "$choice" = "f" ] || [ "$choice" = "F" ]; then
    # Flash the device
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