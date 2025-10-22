# RAMN on Renode (Linux) — CAN Test Guide

## Prerequisites
- Renode installed (Linux)
- can-utils installed: `sudo apt-get install -y can-utils` (or your distro equivalent)

## 1) Create a virtual CAN interface (vcan0)
```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
# Optional: observe traffic
candump vcan0
```

## 2) Start Renode and load the Linux script
- Start Renode, then in the Monitor run:
```bash
(monitor) i @/Users/yusufshaikh/Desktop/Projects/RAMN/firmware/RAMNV1/Debug/ramn_linux.resc
```
This will:
- create machine `ecu0`
- load the Linux platform description
- load `RAMNV1.elf`
- create `canHub0`
- connect `sysbus.fdcan1` and the SocketCAN bridge `socketcan` to the hub
- open a UART analyzer
- start execution

## 3) Send test CAN frames from the host
Use SocketCAN on the host to inject frames into the ECU via `vcan0`:
```bash
# UDS Tester Present to ECU B
cansend vcan0 7E1#3E00

# UDS ReadDataByIdentifier (VIN) to ECU B
cansend vcan0 7E1#22F190
```
Expected: responses on 0x7E9 (and for other ECUs: 0x7EA/0x7EB). You can see them in `candump vcan0` and Renode logs.

## 4) Files used (for reference)
Linux platform (`.repl`):
```1:34:/Users/yusufshaikh/Desktop/Projects/RAMN/firmware/RAMNV1/Debug/stm32l552_ramn_linux.repl
nvic: IRQControllers.NVIC @ sysbus <0xE000E000, +0x1000>

cpu: CPU.CortexM @ sysbus
    cpuType: "cortex-m33"
    nvic: nvic

flash: Memory.MappedMemory @ sysbus 0x08000000 { size: 0x80000 }
sram1: Memory.MappedMemory @ sysbus 0x20000000 { size: 0x40000 }

lpuart1: UART.STM32F7_USART @ sysbus <0x40008000, +0x400> { frequency: 80000000; IRQ -> nvic@70 }

usart1: UART.STM32F7_USART @ sysbus <0x40013800, +0x400> { frequency: 80000000; IRQ -> nvic@37 }
usart3: UART.STM32F7_USART @ sysbus <0x40004800, +0x400> { frequency: 16000000; IRQ -> nvic@39 }

// FDCAN1 Message RAM
fdcan1_msgram: Memory.MappedMemory @ sysbus 0x4000AC00
    size: 0x400

// FDCAN1 Controller
fdcan1: CAN.MCAN @ sysbus <0x4000A400, +0x400>
    messageRAM: fdcan1_msgram
    Line0 -> nvic@19

rcc:       Memory.MappedMemory @ sysbus 0x40021000 { size: 0x400 }
flash_if:  Memory.MappedMemory @ sysbus 0x40022000 { size: 0x400 }
tim1_stub: Memory.MappedMemory @ sysbus 0x40012C00 { size: 0x400 }
pwr:       Memory.MappedMemory @ sysbus 0x40007000 { size: 0x400 }

# Linux SocketCAN bridge (vcan0)
socketcan: CAN.SocketCANBridge @ sysbus
    canInterfaceName: "vcan0"
    ensureFdFrames: false
    ensureXlFrames: false
```

Linux script (`.resc`):
```1:16:/Users/yusufshaikh/Desktop/Projects/RAMN/firmware/RAMNV1/Debug/ramn_linux.resc
mach create "ecu0"

machine LoadPlatformDescription @/Users/yusufshaikh/Desktop/Projects/RAMN/firmware/RAMNV1/Debug/stm32l552_ramn_linux.repl
sysbus LoadELF @/Users/yusufshaikh/Desktop/Projects/RAMN/firmware/RAMNV1/Debug/RAMNV1.elf

# UART analyzer (keep same as mac for consistency)
showAnalyzer sysbus.usart1

# Create CAN hub and connect MCU CAN + SocketCAN bridge
emulation CreateCANHub "canHub0"
connector Connect sysbus.fdcan1 canHub0
connector Connect socketcan canHub0

cpu LogFunctionNames true true
start
```

## 5) Troubleshooting
- No responses: confirm `vcan0` is up and `candump vcan0` shows your `cansend` frames.
- Renode errors on `socketcan`: ensure you’re on Linux and `vcan0` exists.
- Only see `XYZ` on UART: firmware may not have started FreeRTOS; re-run the `.resc` so it loads the ELF and starts.

## 6) References
- Renode UART integration: https://renode.readthedocs.io/en/latest/host-integration/uart.html
- Renode CAN integration (SocketCAN bridge): https://renode.readthedocs.io/en/latest/host-integration/can.html
