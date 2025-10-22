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
