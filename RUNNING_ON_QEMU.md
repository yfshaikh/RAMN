# ✅ RAMN Firmware on QEMU - Setup Complete

## 🚀 You're Ready to Run!

Your system is configured to run RAMN firmware on QEMU. Here's what I've set up for you:

### ✅ What's Ready
- **QEMU**: Version 10.1.1 is installed
- **Firmware**: ECUA.bin is available (166KB)
- **Scripts**: Ready-to-use runner scripts
- **Emulator**: Cortex-M33 machine (matches STM32L552)

### 📋 Quick Start Options

#### Option 1: Basic Run (Simplest)
```bash
chmod +x run_qemu.sh
./run_qemu.sh
```

#### Option 2: Run with Debug Logging
```bash
./run_qemu.sh scripts/firmware/ECUA.bin debug
```

#### Option 3: Test All Machines
```bash
chmod +x test_qemu.sh
./test_qemu.sh
```

#### Option 4: Manual QEMU Command
```bash
qemu-system-arm \
  -M mps2-an505 \
  -cpu cortex-m33 \
  -kernel scripts/firmware/ECUA.bin \
  -nographic \
  -serial stdio
```

Press **Ctrl+A** then **X** to exit QEMU.

## 📚 Documentation Created

I've created these guides for you:

1. **`QUICKSTART_QEMU.md`** - Quick reference and common commands
2. **`QEMU_SETUP_GUIDE.md`** - Comprehensive guide with all options
3. **`run_qemu.sh`** - Main runner script with multiple modes
4. **`test_qemu.sh`** - Test script to try different machines
5. **`ramn.resc`** - Renode script (alternative emulator)
6. **`ramn_custom.repl`** - Custom platform description for Renode

## 🎯 Best Machines for RAMN

RAMN uses **STM32L552CETX** with **Cortex-M33** core. Best QEMU machines:

| Machine | CPU | Match | Notes |
|---------|-----|-------|-------|
| **mps2-an505** | Cortex-M33 | ⭐ Best | Exact CPU match |
| mps2-an521 | Dual M33 | ⭐ Best | Dual core version |
| netduinoplus2 | Cortex-M4 | ✅ Good | More stable |
| olimex-stm32-h405 | Cortex-M4 | ✅ Good | Real STM32 |

The scripts default to **mps2-an505** (Cortex-M33).

## 🔧 Additional Tools (Optional)

### For GDB Debugging
```bash
# Install ARM toolchain
brew install --cask gcc-arm-embedded

# Then use GDB mode
./run_qemu.sh scripts/firmware/ECUA.bin gdb
```

### For Better STM32 Emulation
```bash
# Install Renode (better peripheral support)
brew install --cask renode

# Run with Renode
renode ramn.resc
```

## ⚠️ Important Limitations

Since RAMN is designed for automotive hardware:

### ✅ What WILL Work
- ARM instruction execution
- Memory operations (Flash, RAM)
- Basic code logic
- Function calls
- Interrupt vector loading

### ❌ What WON'T Work
- **CAN/CAN-FD communication** (no peripheral)
- **USB communication** (no peripheral)
- **ADC readings** (no analog input)
- **Real-time behavior** (timing differs)
- **GPIO, SPI, I2C** (limited emulation)
- **UDS/KWP2000/XCP protocols** (need real hardware)

**This is expected!** QEMU is for:
- Code debugging
- Logic testing
- Learning ARM assembly
- Understanding firmware structure

For full RAMN functionality, you need physical hardware or better emulation (Renode).

## 🔍 What to Expect

When you run the firmware:

1. **Initial boot** - Vector table loads, reset handler executes
2. **Initialization** - System and peripheral init
3. **Hang or loop** - Likely waiting for hardware (CAN, USB)

**This is normal!** The firmware expects real automotive hardware.

### Check if It's Working

Good signs:
```
[No output] - Normal, firmware outputs to CAN/USB, not serial
[Hangs]     - Normal, waiting for peripheral events
[No errors] - Good! Code is executing
```

Bad signs:
```
qemu: fatal: Invalid instruction
qemu: fatal: Lockup: can't execute code
Segmentation fault
```

Use debug mode to investigate:
```bash
./run_qemu.sh scripts/firmware/ECUA.bin debug
cat qemu_trace.log
```

## 🎓 Learning & Debugging

### View Execution Trace
```bash
qemu-system-arm \
  -M mps2-an505 \
  -cpu cortex-m33 \
  -kernel scripts/firmware/ECUA.bin \
  -d in_asm,cpu \
  -D trace.log \
  -nographic
```

### Interactive Monitor
```bash
# Terminal 1
qemu-system-arm \
  -M mps2-an505 \
  -cpu cortex-m33 \
  -kernel scripts/firmware/ECUA.bin \
  -monitor telnet:127.0.0.1:55555,server,nowait \
  -nographic

# Terminal 2
telnet localhost 55555
(qemu) info registers
(qemu) x/10i 0x08000000
(qemu) x/10wx 0x20000000
```

### With GDB (if installed)
```bash
# Terminal 1: QEMU
./run_qemu.sh scripts/firmware/ECUA.bin gdb

# Terminal 2: GDB
arm-none-eabi-gdb scripts/firmware/ECUA.bin
(gdb) target remote localhost:1234
(gdb) break main
(gdb) continue
(gdb) step
(gdb) info registers
```

## 🚗 For Real Automotive Testing

QEMU is limited for automotive work. For real testing:

### 1. Virtual CAN Network
```bash
cd scripts/vcand
./run_vcand.sh
```

### 2. Physical RAMN Hardware
- See `hardware/` folder for fabrication files
- Documentation: https://ramn.readthedocs.io/

### 3. CARLA Simulation
```bash
cd scripts/carla
# Follow CARLA setup instructions
./2_CARLA_RAMN_manual_serial.bat
```

## 🐛 Troubleshooting

| Problem | Solution |
|---------|----------|
| Command not found | `chmod +x run_qemu.sh` |
| QEMU not installed | `brew install qemu` |
| Firmware not found | Use: `scripts/firmware/ECUA.bin` |
| GDB not working | Install: `brew install --cask gcc-arm-embedded` |
| Hangs immediately | Normal - try debug mode |
| No output | Normal - output goes to CAN/USB |

## 📖 Next Steps

1. **Try it now**: `./run_qemu.sh`
2. **Read quickstart**: Open `QUICKSTART_QEMU.md`
3. **Learn more**: Open `QEMU_SETUP_GUIDE.md`
4. **Try Renode**: Better STM32 support
5. **Get hardware**: See `hardware/README.md`

## 📞 Resources

- **RAMN Docs**: https://ramn.readthedocs.io/
- **QEMU Docs**: https://www.qemu.org/docs/
- **Renode Docs**: https://renode.readthedocs.io/
- **Project**: /Users/yusufshaikh/Desktop/Projects/RAMN

---

**Ready to run?** Just type: `./run_qemu.sh` 🚀

---

## Memory Layout Reference

For debugging, here's the STM32L552 memory map:

```
0x08000000 - 0x08080000 : Flash (512KB)
0x20000000 - 0x20030000 : SRAM1 (192KB)
0x20030000 - 0x20040000 : SRAM2 (64KB)
0x0BF90000 - 0x0BF98000 : System Memory
0x0BFA0000 - 0x0BFA0200 : OTP
0x40000000 - 0x4FFFFFFF : Peripherals
```

Entry point: Reset handler at `0x08000004`

---

*Note: This is experimental. RAMN is designed for real automotive hardware. QEMU is for learning and basic debugging only.*

