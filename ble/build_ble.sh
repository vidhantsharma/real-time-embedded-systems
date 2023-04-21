# Path to tool-chain
ARMGCC=/opt/arm-gcc

# Tool-chain programs
AR=$ARMGCC/bin/arm-none-eabi-ar

make
rm -f libble_uart.a
$AR -r libble_uart.a _build/nrf52833_xxaa/*.o
$AR -d libble_uart.a gcc_startup_nrf52833.S.o
$AR -d libble_uart.a system_nrf52833.c.o