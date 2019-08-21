################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../startup/startup_stm32.s 

C_SRCS += \
../startup/sysmem.c 

OBJS += \
./startup/startup_stm32.o \
./startup/sysmem.o 

C_DEPS += \
./startup/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
startup/%.o: ../startup/%.s
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Assembler'
	@echo $(PWD)
	arm-none-eabi-as -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -I"/Users/daymoon/Documents/workspace/stm32f10x_peripheral_lib" -g -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

startup/%.o: ../startup/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -DSTM32 -DSTM32F1 -DSTM32F103CBTx -DDEBUG -DUSE_STDPERIPH_DRIVER -I"/Users/daymoon/Documents/workspace/stm32f10x_peripheral_lib" -I"/Users/daymoon/Documents/workspace/Bluepill/inc" -I"/Users/daymoon/Documents/workspace/stm32f10x_peripheral_lib/Libraries/CMSIS/CM3/CoreSupport" -I"/Users/daymoon/Documents/workspace/stm32f10x_peripheral_lib/Libraries/CMSIS/CM3/DeviceSupport/ST/STM32F10x" -I"/Users/daymoon/Documents/workspace/stm32f10x_peripheral_lib/Libraries/STM32F10x_StdPeriph_Driver/inc" -O3 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


