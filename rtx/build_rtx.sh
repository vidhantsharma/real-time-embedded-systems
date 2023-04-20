# Path to tool-chain
ARMGCC=/opt/arm-gcc

# Path to RTX library
RTXDIR=arch

# Tool-chain programs
CC=$ARMGCC/bin/arm-none-eabi-gcc
LD=$ARMGCC/bin/arm-none-eabi-ld
AR=$ARMGCC/bin/arm-none-eabi-ar
SIZE=$ARMGCC/bin/arm-none-eabi-size
OBJCOPY=$ARMGCC/bin/arm-none-eabi-objcopy

# ARM architecture specific compiler flags
ARCH="-mcpu=cortex-m4 -mthumb -mabi=aapcs -mfloat-abi=hard -mfpu=fpv4-sp-d16"

# Keep every function in a separate section, this allows linker to discard unused ones
OPT="-ffunction-sections -fdata-sections -fno-strict-aliasing -fno-builtin"

# Compiler flags
CFLAGS="-I$RTXDIR -O3 -g3 -Wall -Werror $ARCH $OPT"

# Compile the source files
echo Compiling rtx_delay.c
$CC -c $CFLAGS rtx_delay.c
echo Compiling rtx_evflags.c
$CC -c $CFLAGS rtx_evflags.c
echo Compiling rtx_evr.c
$CC -c $CFLAGS rtx_evr.c
echo Compiling rtx_kernel.c
$CC -c $CFLAGS rtx_kernel.c
echo Compiling rtx_lib.c
$CC -c $CFLAGS rtx_lib.c
echo Compiling rtx_memory.c
$CC -c $CFLAGS rtx_memory.c
echo Compiling rtx_mempool.c
$CC -c $CFLAGS rtx_mempool.c
echo Compiling rtx_msgqueue.c
$CC -c $CFLAGS rtx_msgqueue.c
echo Compiling rtx_mutex.c
$CC -c $CFLAGS rtx_mutex.c
echo Compiling rtx_semaphore.c
$CC -c $CFLAGS rtx_semaphore.c
echo Compiling rtx_system.c
$CC -c $CFLAGS rtx_system.c
echo Compiling rtx_thread.c
$CC -c $CFLAGS rtx_thread.c
echo Compiling rtx_timer.c
$CC -c $CFLAGS rtx_timer.c
echo Compiling os_systick.c
$CC -c $CFLAGS os_systick.c
echo Compiling RTX_Config.c
$CC -c $CFLAGS RTX_Config.c
echo Compiling irq_armv7m.S
$CC -c $CFLAGS irq_armv7m.S


# Link the object code to form exectuable program
echo Generating librtx.a
/bin/rm -f librtx.a
$AR -r librtx.a rtx_delay.o rtx_evflags.o rtx_kernel.o \
        rtx_evr.o rtx_lib.o rtx_memory.o rtx_mempool.o \
        rtx_msgqueue.o rtx_mutex.o rtx_semaphore.o \
        rtx_system.o rtx_thread.o rtx_timer.o os_systick.o \
        RTX_Config.o irq_armv7m.o
/bin/rm -f rtx_delay.o rtx_evflags.o rtx_kernel.o \
        rtx_evr.o rtx_lib.o rtx_memory.o rtx_mempool.o \
        rtx_msgqueue.o rtx_mutex.o rtx_semaphore.o \
        rtx_system.o rtx_thread.o rtx_timer.o os_systick.o \
        RTX_Config.o irq_armv7m.o