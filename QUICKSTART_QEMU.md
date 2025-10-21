# RAMN on QEMU - Quick Start Guide

## TL;DR - Run It Now

If you just want to try it immediately:

```bash
# Make sure you have QEMU installed
brew install qemu

# Run the firmware
./run_qemu.sh
```

That's it! The firmware will start running in QEMU.

## Three Options for Emulation

### 1. QEMU (Simplest, Limited)
- ✅ Easy to install and use
- ✅ Good for basic code execution testing
- ❌ No STM32L5-specific peripherals
- ❌ CAN, USB, ADC won't work

### 2. Renode (Better STM32 Support)
- ✅ Better peripheral emulation
- ✅ STM32 platform support
- ✅ Can simulate some peripherals
- ⚠️ Requires additional setup

### 3. Real Hardware (Best)
- ✅ Full functionality
- ✅ Real automotive testing
- ❌ Requires physical RAMN boards

## Quick Commands

### Basic QEMU Execution
```bash
./run_qemu.sh
```

### With Debug Logging
```bash
./run_qemu.sh scripts/firmware/ECUA.bin debug
```

### With GDB Debugging
```bash
# Terminal 1: Start QEMU with GDB server
./run_qemu.sh scripts/firmware/ECUA.bin gdb

# Terminal 2: Connect GDB
arm-none-eabi-gdb scripts/firmware/ECUA.bin
(gdb) target remote localhost:1234
(gdb) break main
(gdb) continue
```

### Test Different ECUs
```bash
# First convert hex to bin
arm-none-eabi-objcopy -I ihex -O binary scripts/firmware/ECUB.hex scripts/firmware/ECUB.bin

# Then run
./run_qemu.sh scripts/firmware/ECUB.bin
```

## Using Renode (Recommended Alternative)

Renode has better STM32 support:

```bash
# Install Renode
brew install --cask renode

# Run with custom script
renode ramn.resc
```

In Renode console:
```
(monitor) start
(monitor) pause
(monitor) cpu PC
(monitor) quit
```

## What Works and What Doesn't

### ✅ What Should Work in QEMU
- Basic ARM Cortex-M code execution
- Memory access (Flash, RAM)
- Simple arithmetic and logic
- Function calls and returns
- Interrupt vector table loading

### ❌ What Won't Work in QEMU
- **CAN/CAN-FD communication** - No CAN peripheral emulation
- **USB communication** - No USB device emulation
- **ADC readings** - No analog input emulation
- **Real-time behavior** - Timing will be different
- **Hardware peripherals** - GPIO, SPI, I2C behavior is limited
- **Automotive protocols** - UDS, KWP2000, XCP require real hardware

## Checking If It's Working

### Signs of Success
1. **QEMU doesn't crash immediately**
2. **No "Invalid instruction" errors**
3. **GDB can connect and set breakpoints**
4. **Code executes past Reset_Handler**

### Common Issues

#### QEMU hangs or crashes
```bash
# Try with debug output
./run_qemu.sh scripts/firmware/ECUA.bin debug
cat qemu_trace.log
```

Look for:
- Invalid memory access
- Unimplemented instructions
- Peripheral access attempts

#### "arm-none-eabi-gdb not found"
```bash
# Install ARM toolchain
brew install --cask gcc-arm-embedded
```

## Manual QEMU Commands

If the script doesn't work, try manually:

```bash
# Basic run
qemu-system-arm \
  -M netduinoplus2 \
  -cpu cortex-m4 \
  -kernel scripts/firmware/ECUA.bin \
  -nographic \
  -serial stdio

# With debugging
qemu-system-arm \
  -M netduinoplus2 \
  -cpu cortex-m4 \
  -kernel scripts/firmware/ECUA.bin \
  -nographic \
  -d guest_errors,unimp \
  -D debug.log
```

## Advanced: Modifying Firmware for QEMU

To make firmware more QEMU-friendly, you can modify the source code:

### 1. Add QEMU flag to build
```bash
cd firmware/RAMNV1
# Edit Makefile or build configuration to add:
# -DQEMU_SIMULATION
```

### 2. Modify peripheral initialization
In source files, add:
```c
#ifdef QEMU_SIMULATION
  // Skip hardware peripheral initialization
  return HAL_OK;
#else
  // Normal hardware init
  HAL_InitPeripheral();
#endif
```

### 3. Mock hardware responses
```c
#ifdef QEMU_SIMULATION
  // Simulate sensor reading
  return 0x42;
#else
  // Read from actual ADC
  return HAL_ADC_GetValue();
#endif
```

## Useful Debug Commands

### In QEMU Monitor (telnet localhost 55555)
```
info registers          # Show CPU registers
info mtree             # Show memory map
x /10wx 0x20000000     # Examine RAM
x /10i 0x08000000      # Disassemble Flash
```

### In GDB
```gdb
# Connect to QEMU
target remote localhost:1234

# Basic debugging
break main
continue
step
next
info registers

# Memory inspection
x/10wx 0x20000000      # Examine RAM
x/10i $pc              # Disassemble at PC
print variable_name

# Flash inspection
x/10i 0x08000000       # First instructions
```

## Expected Behavior

Since RAMN firmware is designed for real automotive hardware, you'll likely see:

1. **Boot and initialization** - Should work
2. **Peripheral setup** - Will attempt but may hang
3. **Main loop** - May hang waiting for hardware events
4. **CAN/USB communication** - Won't work

**This is normal!** QEMU is primarily useful for:
- Testing code logic
- Debugging algorithms
- Learning ARM assembly
- Understanding memory layout

For real RAMN functionality, you need:
- Physical RAMN hardware, OR
- Better emulation (Renode), OR
- Virtual CAN interfaces (see `scripts/vcand/`)

## Next Steps

### For Software Testing
- Use GDB with QEMU for code debugging
- Modify firmware to work in simulation
- Test non-hardware-dependent algorithms

### For Hardware Testing
- Order RAMN boards (see hardware/ folder)
- Use JTAG/SWD debugger
- Follow the full RAMN documentation at https://ramn.readthedocs.io/

### For Virtual CAN Testing
```bash
# Use virtual CAN daemon
cd scripts/vcand
./run_vcand.sh
```

## Troubleshooting

| Issue | Solution |
|-------|----------|
| QEMU not found | `brew install qemu` |
| arm-none-eabi-gdb not found | `brew install --cask gcc-arm-embedded` |
| Firmware hangs | Normal - waiting for hardware. Use GDB to inspect |
| Can't convert hex | Install toolchain: `brew install --cask gcc-arm-embedded` |
| No output | Expected - firmware outputs to CAN/USB, not serial |

## Support

- **RAMN Documentation**: https://ramn.readthedocs.io/
- **QEMU Documentation**: https://www.qemu.org/docs/master/
- **Renode Documentation**: https://renode.readthedocs.io/

For full RAMN functionality, see `QEMU_SETUP_GUIDE.md` for detailed information.

---

**Note**: This is experimental. RAMN is designed for real automotive hardware. QEMU emulation is primarily for educational purposes and basic debugging only.

