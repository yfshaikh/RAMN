# Running RAMN Firmware on QEMU ARM

## Overview

The RAMN firmware is designed for **STM32L552CETX** (ARM Cortex-M33 core) microcontroller. This guide provides instructions for running it on QEMU, though with significant limitations.

## Hardware Specifications

- **MCU**: STM32L552CETX (ARM Cortex-M33 with TrustZone)
- **Flash**: 512KB at address `0x08000000`
- **RAM**: 256KB total (192KB SRAM1 + 64KB SRAM2) at address `0x20000000`
- **Peripherals**: CAN-FD, USB, ADC, SPI, FDCAN, GPIO, etc.

## Important Limitations

⚠️ **QEMU does not have native support for STM32L552**. This means:

1. **No STM32L5-specific peripherals** (CAN-FD, USB, ADC, etc.)
2. **Limited hardware emulation** - Most hardware interactions will fail
3. **No realistic testing** - The firmware is heavily dependent on automotive peripherals
4. **Debugging only** - Mainly useful for basic code execution tracing

## Prerequisites

### Install QEMU (macOS)

```bash
# Install QEMU with ARM support
brew install qemu

# Verify installation
qemu-system-arm --version
```

### Prepare Firmware

The pre-compiled firmware binaries are in `scripts/firmware/`:
- `ECUA.bin` - ECU A binary
- `ECUA.hex` - ECU A hex format
- `ECUB.hex` - ECU B hex format
- `ECUC.hex` - ECU C hex format
- `ECUD.hex` - ECU D hex format

You'll need the `.bin` format for QEMU. If you only have `.hex` files, convert them:

```bash
# Install arm-none-eabi toolchain if needed
brew install --cask gcc-arm-embedded

# Convert hex to bin
arm-none-eabi-objcopy -I ihex -O binary scripts/firmware/ECUA.hex scripts/firmware/ECUA.bin
```

## Method 1: Using Generic Cortex-M Board (Recommended)

QEMU has support for generic ARM Cortex-M boards. This is the simplest approach:

```bash
qemu-system-arm \
  -M netduinoplus2 \
  -cpu cortex-m4 \
  -kernel scripts/firmware/ECUA.bin \
  -nographic \
  -monitor stdio
```

**Explanation**:
- `-M netduinoplus2`: Uses Netduino Plus 2 machine (STM32F405-based, closest available)
- `-cpu cortex-m4`: Cortex-M4 core (close to M33, but without TrustZone)
- `-kernel`: Loads the firmware binary
- `-nographic`: No graphical output
- `-monitor stdio`: QEMU monitor on standard I/O

## Method 2: Using STM32 Boards (Limited Support)

QEMU has limited support for some STM32 boards:

```bash
# List available ARM machines
qemu-system-arm -M help | grep -i stm

# Try STM32F405 (closest to STM32L5)
qemu-system-arm \
  -M netduinoplus2 \
  -cpu cortex-m4 \
  -kernel scripts/firmware/ECUA.bin \
  -serial stdio \
  -nographic
```

## Method 3: Custom Memory Layout

For more control over memory mapping:

```bash
qemu-system-arm \
  -M none \
  -cpu cortex-m33 \
  -kernel scripts/firmware/ECUA.bin \
  -m 256M \
  -nographic \
  -d guest_errors,unimp
```

**Explanation**:
- `-M none`: No machine, just CPU
- `-cpu cortex-m33`: Correct CPU type
- `-d guest_errors,unimp`: Debug unimplemented features

## Method 4: With GDB Debugging

The most useful approach for development - run with GDB attached:

```bash
# Terminal 1: Start QEMU with GDB server
qemu-system-arm \
  -M netduinoplus2 \
  -cpu cortex-m4 \
  -kernel scripts/firmware/ECUA.bin \
  -nographic \
  -S -s

# Terminal 2: Connect with GDB
arm-none-eabi-gdb scripts/firmware/ECUA.bin
(gdb) target remote localhost:1234
(gdb) break main
(gdb) continue
```

**Explanation**:
- `-S`: Freeze CPU at startup
- `-s`: Open GDB server on `localhost:1234`

## Method 5: Using Renode (Better Alternative)

**Renode** has better STM32 support than QEMU. Consider using it instead:

```bash
# Install Renode (macOS)
brew install --cask renode

# Create a Renode script (ramn.resc)
cat > ramn.resc << 'EOF'
# Load STM32L5 platform
mach create "ramn"
machine LoadPlatformDescription @platforms/cpus/stm32l5.repl

# Load firmware
sysbus LoadELF @scripts/firmware/ECUA.bin

# Start
start
EOF

# Run Renode
renode ramn.resc
```

## Debugging Tips

### View CPU Registers

```bash
qemu-system-arm \
  -M netduinoplus2 \
  -cpu cortex-m4 \
  -kernel scripts/firmware/ECUA.bin \
  -d cpu,in_asm \
  -D qemu_trace.log \
  -nographic
```

This creates `qemu_trace.log` with CPU state and disassembly.

### Monitor Memory Access

```bash
qemu-system-arm \
  -M netduinoplus2 \
  -cpu cortex-m4 \
  -kernel scripts/firmware/ECUA.bin \
  -d guest_errors,unimp,cpu_reset \
  -nographic
```

### Interactive Monitor

```bash
qemu-system-arm \
  -M netduinoplus2 \
  -cpu cortex-m4 \
  -kernel scripts/firmware/ECUA.bin \
  -monitor telnet:127.0.0.1:55555,server,nowait \
  -nographic
```

Then connect: `telnet localhost 55555`

Monitor commands:
- `info registers` - Show CPU registers
- `info mtree` - Show memory map
- `x /10wx 0x20000000` - Examine RAM
- `x /10i 0x08000000` - Disassemble Flash

## Expected Issues and Workarounds

### 1. Firmware Hangs on Peripheral Initialization

**Problem**: Firmware waits for hardware that doesn't exist.

**Workaround**:
- Build firmware with `-DQEMU_SIMULATION` flag (requires source modification)
- Skip peripheral initialization code
- Use GDB to manually skip blocking functions

### 2. No USB/CAN Communication

**Problem**: QEMU doesn't emulate STM32 USB or CAN peripherals.

**Workaround**:
- Use Renode instead (has better peripheral support)
- Create mock peripheral behavior
- Focus on testing non-hardware-dependent code

### 3. Reset Handler Issues

**Problem**: Firmware may not start correctly.

**Workaround**:
```bash
# Create ELF file with proper entry point
arm-none-eabi-objcopy \
  -I ihex -O elf32-littlearm \
  --change-start 0x08000000 \
  scripts/firmware/ECUA.hex \
  scripts/firmware/ECUA.elf

qemu-system-arm -M netduinoplus2 -kernel scripts/firmware/ECUA.elf
```

## Building Firmware for QEMU

To make the firmware more QEMU-friendly, modify the source:

### 1. Add QEMU Defines

In `Core/Inc/main.h`:
```c
#ifdef QEMU_SIMULATION
  #define HAL_Delay(x) // No-op for QEMU
#endif
```

### 2. Disable Peripheral Checks

In peripheral initialization functions, add:
```c
#ifndef QEMU_SIMULATION
  // Wait for peripheral ready
  while(!peripheral_ready) {}
#endif
```

### 3. Rebuild

```bash
cd firmware/RAMNV1
# Add -DQEMU_SIMULATION to compiler flags
make clean
make EXTRA_CFLAGS="-DQEMU_SIMULATION"
```

## Alternative: Hardware-in-the-Loop Testing

For realistic testing, consider:

1. **Use actual RAMN hardware** - The boards can be fabricated from the provided Gerber files
2. **JTAG debugging** - Debug on real hardware with OpenOCD
3. **Virtual CAN networks** - Use `vcand` scripts provided in `scripts/vcand/`

## Useful Resources

- QEMU ARM Documentation: https://www.qemu.org/docs/master/system/target-arm.html
- Renode STM32 Support: https://renode.readthedocs.io/
- RAMN Documentation: https://ramn.readthedocs.io/
- OpenOCD for STM32L5: https://openocd.org/

## Quick Start Script

Save this as `run_qemu.sh`:

```bash
#!/bin/bash
set -e

FIRMWARE="${1:-scripts/firmware/ECUA.bin}"

if [ ! -f "$FIRMWARE" ]; then
    echo "Error: Firmware file not found: $FIRMWARE"
    exit 1
fi

echo "Starting QEMU with firmware: $FIRMWARE"
echo "Press Ctrl+A then X to exit"
echo ""

qemu-system-arm \
  -M netduinoplus2 \
  -cpu cortex-m4 \
  -kernel "$FIRMWARE" \
  -nographic \
  -serial stdio \
  -monitor telnet:127.0.0.1:55555,server,nowait \
  -d guest_errors

echo "QEMU exited"
```

Make it executable and run:
```bash
chmod +x run_qemu.sh
./run_qemu.sh scripts/firmware/ECUA.bin
```

## Conclusion

While QEMU can load and execute basic ARM code from the RAMN firmware, **it cannot provide realistic automotive ECU emulation** due to missing peripherals. For proper testing:

- Use **Renode** for better STM32 peripheral emulation
- Use **real RAMN hardware** for authentic testing
- Use **QEMU with GDB** only for basic code flow debugging

The RAMN project is designed to work with physical automotive bus protocols and hardware, making full emulation challenging without the actual hardware.

