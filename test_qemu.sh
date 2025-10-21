#!/bin/bash
# Quick test of RAMN firmware on different QEMU machines

FIRMWARE="scripts/firmware/ECUA.bin"

echo "========================================"
echo "Testing RAMN Firmware on QEMU"
echo "========================================"
echo ""

# Test 1: Cortex-M33 (Best match for STM32L552)
echo "Test 1: MPS2-AN505 (Cortex-M33 - Best Match)"
echo "----------------------------------------"
timeout 3 qemu-system-arm \
  -M mps2-an505 \
  -cpu cortex-m33 \
  -kernel "$FIRMWARE" \
  -nographic \
  -serial stdio \
  -d guest_errors 2>&1 | head -20
echo ""
echo "Test 1 complete (timed out after 3s)"
echo ""

# Test 2: Netduino Plus 2 (Cortex-M4)
echo "Test 2: Netduino Plus 2 (Cortex-M4)"
echo "----------------------------------------"
timeout 3 qemu-system-arm \
  -M netduinoplus2 \
  -cpu cortex-m4 \
  -kernel "$FIRMWARE" \
  -nographic \
  -serial stdio \
  -d guest_errors 2>&1 | head -20
echo ""
echo "Test 2 complete (timed out after 3s)"
echo ""

# Test 3: Olimex STM32-H405 (Real STM32 board)
echo "Test 3: Olimex STM32-H405 (Real STM32)"
echo "----------------------------------------"
timeout 3 qemu-system-arm \
  -M olimex-stm32-h405 \
  -kernel "$FIRMWARE" \
  -nographic \
  -serial stdio \
  -d guest_errors 2>&1 | head -20
echo ""
echo "Test 3 complete (timed out after 3s)"
echo ""

echo "========================================"
echo "Test Summary"
echo "========================================"
echo ""
echo "If you saw minimal errors above, the firmware"
echo "is loading correctly. Choose your preferred machine:"
echo ""
echo "  - mps2-an505: Best match (Cortex-M33)"
echo "  - netduinoplus2: Good compatibility (Cortex-M4)"
echo "  - olimex-stm32-h405: Real STM32 (Cortex-M4)"
echo ""
echo "To run interactively, use:"
echo "  ./run_qemu.sh"

