################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: quadrature_encoder.axf

# Tool invocations
quadrature_encoder.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: MCU Linker'
	arm-none-eabi-gcc -nostdlib -L"/Users/kevinlee/BuggyBuddies/LPC Projects/nxp_lpcxpresso_11c24_board_lib/Debug" -L"/Users/kevinlee/BuggyBuddies/LPC Projects/lpc_chip_11cxx_lib/Debug" -Xlinker -Map="quadrature_encoder.map" -Xlinker --gc-sections -Xlinker -print-memory-usage -mcpu=cortex-m0 -mthumb -T "quadrature_encoder_Debug.ld" -o "quadrature_encoder.axf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(EXECUTABLES)$(OBJS)$(C_DEPS) quadrature_encoder.axf
	-@echo ' '

post-build:
	-@echo 'Performing post-build steps'
	-arm-none-eabi-size "quadrature_encoder.axf"; # arm-none-eabi-objcopy -v -O binary "quadrature_encoder.axf" "quadrature_encoder.bin" ; # checksum -p LPC11C24_301 -d "quadrature_encoder.bin";
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY: post-build

-include ../makefile.targets
