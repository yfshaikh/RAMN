/*
 * FreeRTOS Port Wrapper for RAMN
 *
 * This wrapper conditionally includes the appropriate FreeRTOS port
 * based on the build configuration:
 * - RENODE_SIM: Uses ARM_CM3 port for compatibility with Renode emulator
 * - Normal build: Uses ARM_CM33_NTZ port for STM32L552 hardware
 */

#ifndef FREERTOS_PORT_WRAPPER_H
#define FREERTOS_PORT_WRAPPER_H

#ifdef RENODE_SIM
/* Use ARM_CM3 port for Renode compatibility */
/* ARM_CM3 port avoids ARMv8-M registers that Renode doesn't implement */
#include "../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3/portmacro.h"
#else
/* Use ARM_CM33_NTZ port for real hardware */
#include "../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM33_NTZ/non_secure/portmacro.h"
#endif

#endif /* FREERTOS_PORT_WRAPPER_H */
