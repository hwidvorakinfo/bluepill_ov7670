################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/camera.c \
../src/delay.c \
../src/gpio.c \
../src/i2c.c \
../src/ili9341.c \
../src/leds.c \
../src/main.c \
../src/mcuperipherals.c \
../src/ov7670.c \
../src/scheduler.c \
../src/services.c \
../src/spi.c \
../src/stm32f10x_it.c \
../src/system_stm32f10x.c \
../src/timer.c 

OBJS += \
./src/camera.o \
./src/delay.o \
./src/gpio.o \
./src/i2c.o \
./src/ili9341.o \
./src/leds.o \
./src/main.o \
./src/mcuperipherals.o \
./src/ov7670.o \
./src/scheduler.o \
./src/services.o \
./src/spi.o \
./src/stm32f10x_it.o \
./src/system_stm32f10x.o \
./src/timer.o 

C_DEPS += \
./src/camera.d \
./src/delay.d \
./src/gpio.d \
./src/i2c.d \
./src/ili9341.d \
./src/leds.d \
./src/main.d \
./src/mcuperipherals.d \
./src/ov7670.d \
./src/scheduler.d \
./src/services.d \
./src/spi.d \
./src/stm32f10x_it.d \
./src/system_stm32f10x.d \
./src/timer.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -DSTM32 -DSTM32F1 -DSTM32F103CBTx -DDEBUG -DUSE_STDPERIPH_DRIVER -I"/Users/daymoon/Documents/workspace/stm32f10x_peripheral_lib" -I"/Users/daymoon/Documents/workspace/Bluepill/inc" -I"/Users/daymoon/Documents/workspace/stm32f10x_peripheral_lib/Libraries/CMSIS/CM3/CoreSupport" -I"/Users/daymoon/Documents/workspace/stm32f10x_peripheral_lib/Libraries/CMSIS/CM3/DeviceSupport/ST/STM32F10x" -I"/Users/daymoon/Documents/workspace/stm32f10x_peripheral_lib/Libraries/STM32F10x_StdPeriph_Driver/inc" -O3 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


