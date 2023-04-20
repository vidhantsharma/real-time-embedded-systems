# Path to tool-chain
ARMGCC=/opt/arm-gcc
export PATH=$ARMGCC/bin:"$PATH"

# Path to BSP library
BSPDIR=bsp

# Path to BLE library
BLEDIR=ble

# Path to RTX library
RTXDIR=rtx

# ARM architecture specific compiler flags
ARCH="-mcpu=cortex-m4 -mthumb -mabi=aapcs -mfloat-abi=hard -mfpu=fpv4-sp-d16"

# Keep every function in a separate section, this allows linker to discard unused ones
OPT="-ffunction-sections -fdata-sections -fno-strict-aliasing -fno-builtin"

# Compiler flags
CFLAGS="-I$BSPDIR -I$BLEDIR -I$RTXDIR -O3 -g3 -Wall -Werror $ARCH $OPT"

# Compile the source files
echo Compiling main.c
arm-none-eabi-gcc -c $CFLAGS main.c

# Link the object code to form exectuable program
echo Linking MICROBIT.out
arm-none-eabi-ld -T ble_uart.ld -Map system.map \
        --entry Reset_Handler --gc-sections \
        main.o $BSPDIR/libbsp.a $BLEDIR/libble_uart.a $RTXDIR/librtx.a \
        $ARMGCC/arm-none-eabi/lib/thumb/v7e-m+fp/hard/libm.a \
        $ARMGCC/arm-none-eabi/lib/thumb/v7e-m+fp/hard/libc.a \
        $ARMGCC/lib/gcc/arm-none-eabi/12.2.1/thumb/v7e-m+fp/hard/libgcc.a \
        $ARMGCC/arm-none-eabi/lib/thumb/v7e-m+fp/hard/libnosys.a \
        -o app.out

# Check sizes
arm-none-eabi-size app.out

# Generate HEX file to load on the target
arm-none-eabi-objcopy -O ihex app.out app.hex
mergehex -m $BLEDIR/hex/s113_nrf52_7.2.0_softdevice.hex app.hex -o MICROBIT.hex

# Upload on the target
cmd.exe /c copy 'U:\home\wicked\RTES23\Final_project\MICROBIT.hex' Q:

# The above command is applicable for a Windows PC. You may change it
# appropriately to on Linux or MacOS. It should be something like:
# cp MICROBIT.hex /media/MICROBIT       # linux
# cp MICROBIT.hex /Volumes/MICROBIT     # mac
