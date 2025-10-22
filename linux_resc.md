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
