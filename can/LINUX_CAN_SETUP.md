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
In the Renode monitor:
```bash
(monitor) emulation CreateCANHub "ramn_net"
(monitor) connector Connect sysbus.fdcan1 ramn_net
(monitor) connector Connect ramn_net socketcan:vcan0
```

Notes:
- If your Renode expects a different URI, use `socketcan:can0` and create `can0` instead of `vcan0`.
- Ensure the platform maps MCAN correctly:
  - `fdcan1: CAN.MCAN @ sysbus <0x4000A400, +0x400>`
  - `fdcan1_msgram: Memory.MappedMemory @ sysbus 0x4000AC00 { size: 0x400 }`
  - `Line0 -> nvic@19`

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
- No frames visible: check Renode connections with `connector ShowConnections`.
- Flood of function logs: disable with `(monitor) cpu LogFunctionNames false`.
- For real hardware-like `canX` devices (not vcan): set bitrate before `up`:
```bash
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0
```
- If UART logs are missing, ignore; CAN I/O is independent of UART/printf.


