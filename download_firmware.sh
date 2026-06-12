#!/bin/bash
# CF-Drone 固件下载与编译脚本
# 使用方法: chmod +x download_firmware.sh && ./download_firmware.sh

set -e

echo "============================================"
echo "  CF-Drone 固件编译脚本"
echo "  目标: ESP32 / ESP32-S3 / ESP32-C3"
echo "============================================"

# 检查 arduino-cli
if ! command -v arduino-cli &> /dev/null; then
    echo "[1/5] 安装 arduino-cli..."
    curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=/usr/local/bin sh
else
    echo "[1/5] arduino-cli 已安装 ✓"
fi

# 初始化配置
echo "[2/5] 配置 ESP32 开发板..."
arduino-cli config init 2>/dev/null || true
arduino-cli config add board_manager.additional_urls https://espressif.github.io/arduino-esp32/package_esp32_index.json 2>/dev/null || true
arduino-cli core update-index 2>/dev/null || true

# 安装 ESP32 核心
echo "[3/5] 安装 ESP32 核心库 (首次安装需要下载约1GB，请耐心等待)..."
arduino-cli core install esp32:esp32

# 安装 MAVLink 库
echo "[4/5] 安装 MAVLink 库..."
arduino-cli lib install MAVLink 2>/dev/null || true

# 编译
echo "[5/5] 编译固件..."
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# ESP32
echo ""
echo ">>> 编译 ESP32 固件..."
arduino-cli compile \
    --fqbn esp32:esp32:esp32 \
    --build-path "$SCRIPT_DIR/firmware/esp32" \
    "$SCRIPT_DIR"

# ESP32-S3
echo ">>> 编译 ESP32-S3 固件..."
mkdir -p "$SCRIPT_DIR/firmware/esp32s3"
arduino-cli compile \
    --fqbn esp32:esp32:esp32s3 \
    --build-path "$SCRIPT_DIR/firmware/esp32s3" \
    "$SCRIPT_DIR"

# ESP32-C3
echo ">>> 编译 ESP32-C3 固件..."
mkdir -p "$SCRIPT_DIR/firmware/esp32c3"
arduino-cli compile \
    --fqbn esp32:esp32:esp32c3 \
    --build-path "$SCRIPT_DIR/firmware/esp32c3" \
    "$SCRIPT_DIR"

echo ""
echo "============================================"
echo "  编译完成！固件位置："
echo ""
echo "  ESP32:   firmware/esp32/CF-Drone.ino.bin"
echo "  ESP32-S3: firmware/esp32s3/CF-Drone.ino.bin"
echo "  ESP32-C3: firmware/esp32c3/CF-Drone.ino.bin"
echo ""
echo "  烧录命令 (以 ESP32 为例)："
echo "  esptool.py --chip esp32 --port /dev/ttyUSB0 write_flash -z 0x0 firmware/esp32/CF-Drone.ino.merged.bin"
echo ""
echo "  分文件烧录："
echo "  esptool.py --chip esp32 --port /dev/ttyUSB0 write_flash -z \\"
echo "    0x1000 firmware/esp32/CF-Drone.ino.bootloader.bin \\"
echo "    0x8000 firmware/esp32/CF-Drone.ino.partitions.bin \\"
echo "    0xe000 firmware/esp32/boot_app0.bin \\"
echo "    0x10000 firmware/esp32/CF-Drone.ino.bin"
echo "============================================"
