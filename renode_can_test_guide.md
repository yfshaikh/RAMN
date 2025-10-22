# Testing CAN Communication Directly in Renode

## 🚀 **Immediate Testing - No Additional Setup Required**

### **Step 1: Start Your ECU**
```bash
cd /Users/yusufshaikh/Desktop/Projects/RAMN/firmware/RAMNV1/Debug
renode ramn_mac.resc
```

### **Step 2: Test CAN Communication**

#### **A. Send UDS Diagnostic Messages**
```bash
# UDS Read Data By Identifier (Service 0x22)
(monitor) sysbus.fdcan1 SendFrame [0x7E1] [0x22, 0xF1, 0x90]

# UDS Tester Present (Service 0x3E)  
(monitor) sysbus.fdcan1 SendFrame [0x7E1] [0x3E, 0x00]

# UDS Session Control (Service 0x10)
(monitor) sysbus.fdcan1 SendFrame [0x7E1] [0x10, 0x85]
```

#### **B. Send KWP2000 Messages**
```bash
# KWP2000 Tester Present
(monitor) sysbus.fdcan1 SendFrame [0x7E5] [0x3E, 0x00] 

# KWP2000 Start Diagnostic Session
(monitor) sysbus.fdcan1 SendFrame [0x7E5] [0x10, 0x85]
```

#### **C. Send XCP Messages**
```bash
# XCP Connect Command
(monitor) sysbus.fdcan1 SendFrame [0x552] [0xFF, 0x00]

# XCP Get ID Command  
(monitor) sysbus.fdcan1 SendFrame [0x552] [0xFA, 0x00]
```

### **Step 3: Monitor Responses**
```bash
# Enable detailed CAN logging
(monitor) logLevel 3 sysbus.fdcan1

# Watch for responses on TX IDs:
# UDS: 0x7E9 (ECU B), 0x7EA (ECU C), 0x7EB (ECU D)
# XCP: 0x553 (ECU B), 0x555 (ECU C), 0x557 (ECU D)  
# KWP: 0x7ED (ECU B), 0x7EE (ECU C), 0x7EF (ECU D)
```

### **Step 4: Expected Results**
- **Positive Response**: Response with appropriate service ID + 0x40
- **Negative Response**: 0x7F + Service ID + Error Code
- **No Response**: Check ECU is running and CAN ID is correct

### **Example Success Output:**
```
[INFO] sysbus.fdcan1: Received frame ID=0x7E1 Data=[22, F1, 90]
[INFO] sysbus.fdcan1: Sending frame ID=0x7E9 Data=[62, F1, 90, 52, 41, 4D, 4E]
```

This means: "ECU B responded to Read Data request with data: RAMN"
