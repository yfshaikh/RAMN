# Testing CAN Communication with RAMN in Renode

## 1. Start Your ECU (if not already running)
```bash
cd /Users/yusufshaikh/Desktop/Projects/RAMN/firmware/RAMNV1/Debug
renode ramn_mac.resc
```

## 2. Test CAN Message Transmission
In the Renode monitor, try these commands:

```bash
# Send a CAN frame to your ECU
(monitor) sysbus.fdcan1 SendFrame [0x123] [0x01, 0x02, 0x03, 0x04]

# Send diagnostic messages (UDS format)
(monitor) sysbus.fdcan1 SendFrame [0x7E1] [0x10, 0x03, 0x22, 0xF1, 0x90]

# Monitor CAN traffic by enabling logging
(monitor) logLevel 3 sysbus.fdcan1

# Create a CAN hub for multi-ECU simulation
(monitor) emulation CreateCANHub "automotive_network"
(monitor) connector Connect sysbus.fdcan1 automotive_network
```

## 3. Expected Results
- Your ECU should respond to diagnostic messages
- You'll see CAN traffic in the logs
- FDCAN controller will show message processing activity

## 4. What CAN IDs to Use
Based on RAMN documentation:

**UDS (Unified Diagnostic Services):**
- ECU A: RX=0x7E0, TX=0x7E8
- ECU B: RX=0x7E1, TX=0x7E9  
- ECU C: RX=0x7E2, TX=0x7EA
- ECU D: RX=0x7E3, TX=0x7EB

**XCP (Universal Measurement and Calibration Protocol):**
- ECU B: RX=0x552, TX=0x553
- ECU C: RX=0x554, TX=0x555
- ECU D: RX=0x556, TX=0x557

**KWP2000:**
- ECU B: RX=0x7E5, TX=0x7ED
- ECU C: RX=0x7E6, TX=0x7EE  
- ECU D: RX=0x7E7, TX=0x7EF
