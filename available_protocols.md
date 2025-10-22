# RAMN Automotive Protocols Reference

## 🚗 What You Can Do With Your ECU

### **1. UDS (Unified Diagnostic Services) - ISO 14229**
**Most modern automotive protocol used by all major manufacturers**

**Available Scripts:**
- `UDS_PrintInfo.py` - Read ECU information (VIN, firmware version, serial numbers)
- `UDS_DumpMemory.py` - Extract complete ECU memory (flash/RAM) for analysis  
- `UDS_Reprogram.py` - Flash new firmware to ECU
- `UDS_WriteVIN.py` - Write Vehicle Identification Number
- `UDS_LoadTest.py` - High-speed communication testing
- `UDS_Verify.py` - Verify firmware integrity
- `UDS_Chip8.py` - Run Chip-8 games on ECU display!
- `UDS_DisplayImage.py` - Display images on ECU

**CAN IDs Used:**
- ECU A: 0x7E0 → 0x7E8  
- ECU B: 0x7E1 → 0x7E9
- ECU C: 0x7E2 → 0x7EA  
- ECU D: 0x7E3 → 0x7EB

### **2. KWP2000 (Keyword Protocol 2000) - ISO 14230**
**Legacy protocol still used in many vehicles**

**Available Scripts:**
- `KWP_TesterPresent.py` - Keep diagnostic session alive
- `KWP_DisableNormalTransmission.py` - Stop ECU periodic messages
- `KWP_EnableNormalTransmission.py` - Resume ECU periodic messages

**CAN IDs Used:**
- ECU B: 0x7E5 → 0x7ED
- ECU C: 0x7E6 → 0x7EE
- ECU D: 0x7E7 → 0x7EF

### **3. XCP (Universal Measurement and Calibration Protocol)**
**Used for real-time data acquisition and calibration**

**Available Scripts:**
- `XCP_ReadECUInfo.py` - Read XCP-specific ECU information
- `XCP_DumpMemory.py` - Memory dumping via XCP protocol

**CAN IDs Used:**
- ECU B: 0x552 → 0x553
- ECU C: 0x554 → 0x555  
- ECU D: 0x556 → 0x557

## 🎮 **Fun Demos You Can Try:**

### **Chip-8 Gaming on ECU**
```bash
source ramn_env/bin/activate
cd diagnostics
python UDS_Chip8.py
```

### **Display Custom Images**  
```bash
python UDS_DisplayImage.py
```

### **Extract Complete ECU Firmware**
```bash  
python UDS_DumpMemory.py
```

### **Read All ECU Information**
```bash
python UDS_PrintInfo.py
```

## 🔧 **Connection Methods:**

### **Method 1: Direct USB to ECU A**
- ECU A acts as CAN-USB bridge
- All other ECUs (B,C,D) communicate via CAN bus
- Scripts auto-detect USB port

### **Method 2: SocketCAN Interface** 
- Use `python-can` with SocketCAN backend
- Create virtual CAN interface: `vcan0`
- More realistic for multi-ECU testing

### **Method 3: Renode CAN Hub**
- Pure emulation environment
- Connect multiple virtual ECUs
- Perfect for protocol development

## 🚀 **Next Level: Multi-ECU Networks**

Create automotive networks with:
- Engine Control Module (ECM) 
- Body Control Module (BCM)
- Anti-lock Braking System (ABS)
- Airbag Control Module
- Infotainment System

Each running different automotive protocols!
