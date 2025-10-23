### Linux CAN setup for RAMN on Renode (SocketCAN + python-can)

This guide connects Renode's FDCAN (MCAN) to a Linux virtual CAN (vcan) interface so your actuator/diagnostic scripts can send/receive frames using `python-can`.

#### 1) Install prerequisites
```bash
sudo apt update
sudo apt install -y can-utils python3-pip
pip3 install --upgrade pip
pip3 install python-can
```

#### 2) Create and enable a virtual CAN interface
```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

#### 3) Launch Renode and bridge FDCAN to SocketCAN

The platform definition (`.repl`) should already include the SocketCAN bridge. For example, in your `stm32l552_ramn_linux.repl`:

```repl
# FDCAN1 controller
fdcan1: CAN.MCAN @ sysbus <0x4000A400, +0x400>
    messageRAM: fdcan1_msgram
    Line0 -> nvic@19

# SocketCAN bridge (connects to host vcan0)
socketcan: CAN.SocketCANBridge @ sysbus
    canInterfaceName: "vcan0"
    ensureFdFrames: false
    ensureXlFrames: false
```

Then in your Renode script (`.resc`), create the CAN hub and connect both the MCU and the bridge:

```bash
# Create CAN hub
(monitor) emulation CreateCANHub "canHub0"

# Load platform with socketcan bridge
(monitor) machine LoadPlatformDescription @/path/to/stm32l552_ramn_linux.repl
(monitor) sysbus LoadELF @/path/to/RAMNV1.elf

# Connect MCU FDCAN to hub
(monitor) connector Connect sysbus.fdcan1 canHub0

# Connect SocketCAN bridge to hub
(monitor) connector Connect sysbus.socketcan canHub0

# Start execution
(monitor) start
```

**Notes:**
- If `socketcan` is already defined in your `.repl` file, you don't need to create it manually
- The bridge automatically connects to the `vcan0` interface defined in the `.repl`
- Ensure the platform maps MCAN correctly with proper messageRAM and IRQ settings

#### 4) Quick sanity checks
- Observe traffic:
```bash
candump vcan0
```
- Send a test frame from the host:
```bash
cansend vcan0 123#01020304
```

#### 5) Example actuator script (python-can)
```python
import can

bus = can.interface.Bus(interface='socketcan', channel='vcan0')

# Stimulus (sensor/actuator frame)
bus.send(can.Message(arbitration_id=0x123, data=bytes([1,2,3,4]), is_extended_id=False))

# Read ECU response
msg = bus.recv(timeout=1.0)
if msg:
    print(f"RX id=0x{msg.arbitration_id:X} data={msg.data.hex()}")
else:
    print("No response")
```

#### 6) Troubleshooting
- **"Given name 'socketcan' is already used"**: The bridge is already defined in your `.repl` file. Don't try to create it again.
- **No frames visible**: Check Renode connections with `(monitor) connector ShowConnections`
- **Flood of function logs**: Disable with `(monitor) cpu LogFunctionNames false`
- **For real hardware-like `canX` devices** (not vcan): Set bitrate before `up`:
```bash
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0
```
- **If UART logs are missing**: Ignore; CAN I/O is independent of UART/printf

#### 7) References
- [Renode CAN Integration (SocketCAN bridge)](https://renode.readthedocs.io/en/latest/host-integration/can.html)
- [Antmicro: Demonstrating CAN support in Renode with SocketCAN and Wireshark](https://antmicro.com/blog/2024/11/demonstrating-can-support-in-renode/)


