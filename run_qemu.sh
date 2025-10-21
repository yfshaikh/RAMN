#!/bin/bash
# RAMN Firmware QEMU Runner
# This script runs RAMN firmware on QEMU ARM emulator

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Default firmware
FIRMWARE="${1:-scripts/firmware/ECUA.bin}"
MODE="${2:-basic}"

print_usage() {
    echo "Usage: $0 [firmware] [mode]"
    echo ""
    echo "Firmware options:"
    echo "  scripts/firmware/ECUA.bin (default)"
    echo "  scripts/firmware/ECUB.hex"
    echo "  scripts/firmware/ECUC.hex"
    echo "  scripts/firmware/ECUD.hex"
    echo ""
    echo "Mode options:"
    echo "  basic    - Basic execution (default)"
    echo "  debug    - Debug mode with trace logging"
    echo "  gdb      - GDB server mode (connect with arm-none-eabi-gdb on port 1234)"
    echo "  monitor  - With telnet monitor on port 55555"
    echo ""
    echo "Examples:"
    echo "  $0                                    # Run ECUA.bin in basic mode"
    echo "  $0 scripts/firmware/ECUA.bin debug   # Run with debug logging"
    echo "  $0 scripts/firmware/ECUA.bin gdb     # Run with GDB server"
}

# Check if QEMU is installed
if ! command -v qemu-system-arm &> /dev/null; then
    echo -e "${RED}Error: qemu-system-arm not found${NC}"
    echo "Install with: brew install qemu"
    exit 1
fi

# Check firmware file
if [ ! -f "$FIRMWARE" ]; then
    echo -e "${RED}Error: Firmware file not found: $FIRMWARE${NC}"
    echo ""
    print_usage
    exit 1
fi

# Convert hex to bin if needed
FIRMWARE_BIN="$FIRMWARE"
if [[ "$FIRMWARE" == *.hex ]]; then
    FIRMWARE_BIN="${FIRMWARE%.hex}.bin"
    
    if [ ! -f "$FIRMWARE_BIN" ]; then
        echo -e "${YELLOW}Converting HEX to BIN format...${NC}"
        
        if ! command -v arm-none-eabi-objcopy &> /dev/null; then
            echo -e "${RED}Error: arm-none-eabi-objcopy not found${NC}"
            echo "Install with: brew install --cask gcc-arm-embedded"
            exit 1
        fi
        
        arm-none-eabi-objcopy -I ihex -O binary "$FIRMWARE" "$FIRMWARE_BIN"
        echo -e "${GREEN}Converted to: $FIRMWARE_BIN${NC}"
    fi
fi

echo -e "${GREEN}╔════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║  RAMN Firmware QEMU Emulator              ║${NC}"
echo -e "${GREEN}╚════════════════════════════════════════════╝${NC}"
echo ""
echo -e "Firmware: ${YELLOW}$FIRMWARE_BIN${NC}"
echo -e "Mode:     ${YELLOW}$MODE${NC}"
echo ""
echo -e "${YELLOW}⚠️  Note: QEMU has limited STM32L5 support${NC}"
echo -e "${YELLOW}   Most peripherals (CAN, USB, ADC) will not work${NC}"
echo ""

# Base QEMU command
# Using MPS2-AN505 with Cortex-M33 (best match for STM32L552)
# Alternative: netduinoplus2 with cortex-m4 (more stable but less accurate)
BASE_CMD="qemu-system-arm -M mps2-an505 -cpu cortex-m33 -kernel $FIRMWARE_BIN"

case "$MODE" in
    basic)
        echo -e "${GREEN}Starting in basic mode...${NC}"
        echo "Press Ctrl+A then X to exit"
        echo ""
        $BASE_CMD -nographic -serial stdio
        ;;
        
    debug)
        echo -e "${GREEN}Starting in debug mode...${NC}"
        echo "Debug log will be saved to: qemu_trace.log"
        echo "Press Ctrl+A then X to exit"
        echo ""
        $BASE_CMD -nographic -serial stdio \
            -d guest_errors,unimp,cpu_reset \
            -D qemu_trace.log
        echo ""
        echo -e "${GREEN}Debug log saved to: qemu_trace.log${NC}"
        ;;
        
    gdb)
        echo -e "${GREEN}Starting GDB server mode...${NC}"
        echo ""
        echo -e "${YELLOW}Connect with GDB using:${NC}"
        echo "  arm-none-eabi-gdb $FIRMWARE_BIN"
        echo "  (gdb) target remote localhost:1234"
        echo "  (gdb) break main"
        echo "  (gdb) continue"
        echo ""
        echo "QEMU will wait for GDB connection on port 1234..."
        echo "Press Ctrl+C to stop"
        echo ""
        $BASE_CMD -nographic -S -s
        ;;
        
    monitor)
        echo -e "${GREEN}Starting with telnet monitor...${NC}"
        echo ""
        echo -e "${YELLOW}Monitor available at:${NC}"
        echo "  telnet localhost 55555"
        echo ""
        echo "Press Ctrl+A then X to exit"
        echo ""
        $BASE_CMD -nographic -serial stdio \
            -monitor telnet:127.0.0.1:55555,server,nowait
        ;;
        
    *)
        echo -e "${RED}Error: Unknown mode: $MODE${NC}"
        echo ""
        print_usage
        exit 1
        ;;
esac

echo ""
echo -e "${GREEN}QEMU session ended${NC}"

