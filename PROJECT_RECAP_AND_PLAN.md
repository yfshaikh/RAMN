### RAMN on Renode — Project Recap, HAL behavior, and Next Steps

#### Goal
- Run RAMN STM32L5 firmware fully in Renode for practical development and testing: FreeRTOS tasks, HAL-based FDCAN, optional UART debug, and the ability to drive actuators and read ECU responses over CAN from host scripts.

#### Current state
- FreeRTOS: running in Renode; scheduler and tasks execute normally.
- FDCAN: mapped to MCAN in the platform; message RAM configured; connected to a Renode CAN hub.
- UART: raw register writes print (e.g., "XYZ"); HAL/printf output is optional and can work if UART clocking matches the platform.
- Logging: function-name spam was due to `cpu LogFunctionNames true` and can be disabled.

#### Why some UART messages did not print
- HAL/printf depends on a valid clock tree and a correct UART BRR. Under `RENODE_SIM`, complex RCC/CRS/PLL init is skipped to avoid touching registers Renode does not fully emulate. That mismatch made `HAL_UART_Transmit`/`printf` invisible to the analyzer, while direct `USART1->TDR` writes still worked.
- Mitigation in place: select a known USART1 clock (HSI 16 MHz) under `RENODE_SIM` and match the Renode `.repl` `usart1` frequency to 16 MHz. This allows HAL/printf to work if desired.

#### Do you need UART prints for the workflow?
- No. The architecture communicates over CAN. As long as FDCAN initializes and is connected to a CAN hub, actuator scripts can inject frames and the ECU will respond. UART is only for debugging convenience.

#### Known gaps vs. real hardware (acceptable for this goal)
- Detailed RCC/CRS/PLL behavior, USB device stack, TRNG, and some analog/DMA interactions are not fully modeled in Renode. These can remain disabled without impacting CAN/diagnostics development.

#### Next steps
- Keep HAL + FreeRTOS + FDCAN enabled; leave USB/TRNG/complex clocks disabled in sim.
- Ensure `fdcan1` and its message RAM are mapped at `0x4000A400` and `0x4000AC00` with size `0x400`, and `Line0 -> nvic@19` in the `.repl`.
- In the `.resc`, create a CAN hub and connect `sysbus.fdcan1` to it. Bridge that hub to the host (SocketCAN/vcan on Linux, or a small custom python-can driver on macOS).
- Drive actuator scenarios from a python-can daemon and verify ECU responses. Iterate on diagnostics/features purely in software.
- Optional: keep HAL/printf working by maintaining the HSI-based UART clock and the `.repl` frequency alignment.


