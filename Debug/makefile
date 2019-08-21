################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include startup/subdir.mk
-include src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: Bluepill.elf

dependents:
	-cd /Users/daymoon/Documents/workspace/stm32f10x_peripheral_lib/Debug && $(MAKE) all

# Tool invocations
Bluepill.elf: $(OBJS) $(USER_OBJS) /Users/daymoon/Documents/workspace/stm32f10x_peripheral_lib/Debug/libstm32f10x_peripheral_lib.a /Users/daymoon/Documents/workspace/Bluepill/LinkerScript.ld
	@echo 'Building target: $@'
	@echo 'Invoking: MCU GCC Linker'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -L"/Users/daymoon/Documents/workspace/stm32f10x_peripheral_lib/Debug" -T"/Users/daymoon/Documents/workspace/Bluepill/LinkerScript.ld" -Wl,-Map=output.map -Wl,--gc-sections -o "Bluepill.elf" @"objects.list" $(USER_OBJS) $(LIBS) -lm
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

post-build:
	-@echo 'Generating binary and Printing size information:'
	arm-none-eabi-objcopy -O binary "Bluepill.elf" "Bluepill.bin"
	arm-none-eabi-size "Bluepill.elf"
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY: post-build
/Users/daymoon/Documents/workspace/stm32f10x_peripheral_lib/Debug/libstm32f10x_peripheral_lib.a:

-include ../makefile.targets
