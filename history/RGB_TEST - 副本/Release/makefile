################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
RGB_TEST_slave.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
RGB_TEST_slave.list \

OBJCOPY_BIN += \
RGB_TEST_slave.bin \


# All Target
all: RGB_TEST_slave.elf secondary-outputs

# Tool invocations
RGB_TEST_slave.elf: $(OBJS) $(USER_OBJS) E:\STM32Cube\RGB_TEST\STM32F103RDTX_FLASH.ld
	arm-none-eabi-gcc -o "RGB_TEST_slave.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"E:\STM32Cube\RGB_TEST\STM32F103RDTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="RGB_TEST_slave.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

RGB_TEST_slave.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "RGB_TEST_slave.list"
	@echo 'Finished building: $@'
	@echo ' '

RGB_TEST_slave.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "RGB_TEST_slave.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
