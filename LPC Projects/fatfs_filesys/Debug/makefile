################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
-include source/subdir.mk
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
all: fatfs_filesys.axf

# Tool invocations
fatfs_filesys.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: MCU Linker'
	arm-none-eabi-gcc -nostdlib -L"/Users/vivaanbahl/Buggy/BuggyBuddies/LPC Projects/nxp_lpcxpresso_11c24_board_lib/Debug" -L"/Users/vivaanbahl/Buggy/BuggyBuddies/LPC Projects/lpc_chip_11cxx_lib/Debug" -Xlinker -Map="fatfs_filesys.map" -Xlinker --gc-sections -Xlinker -print-memory-usage -mcpu=cortex-m0 -mthumb -T "fatfs_filesys_Debug.ld" -o "fatfs_filesys.axf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(EXECUTABLES)$(OBJS)$(C_DEPS) fatfs_filesys.axf
	-@echo ' '

post-build:
	-@echo 'Performing post-build steps'
	-arm-none-eabi-size "fatfs_filesys.axf"; # arm-none-eabi-objcopy -v -O binary "fatfs_filesys.axf" "fatfs_filesys.bin" ; # checksum -p LPC11C24_301 -d "fatfs_filesys.bin";
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY: post-build

-include ../makefile.targets
