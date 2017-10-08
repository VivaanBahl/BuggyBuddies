################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/Users/vivaanbahl/Buggy/BuggyBuddies/LPC\ Libraries/ff13/source/diskio.c \
/Users/vivaanbahl/Buggy/BuggyBuddies/LPC\ Libraries/ff13/source/ff.c \
/Users/vivaanbahl/Buggy/BuggyBuddies/LPC\ Libraries/ff13/source/ffsystem.c \
/Users/vivaanbahl/Buggy/BuggyBuddies/LPC\ Libraries/ff13/source/ffunicode.c 

OBJS += \
./source/diskio.o \
./source/ff.o \
./source/ffsystem.o \
./source/ffunicode.o 

C_DEPS += \
./source/diskio.d \
./source/ff.d \
./source/ffsystem.d \
./source/ffunicode.d 


# Each subdirectory must supply rules for building sources it contributes
source/diskio.o: /Users/vivaanbahl/Buggy/BuggyBuddies/LPC\ Libraries/ff13/source/diskio.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -DDEBUG -D__CODE_RED -DCORE_M0 -D__USE_LPCOPEN -D__LPC11XX__ -D__REDLIB__ -I"/Users/vivaanbahl/Buggy/BuggyBuddies/LPC Projects/fatfs_filesys/inc" -I"/Users/vivaanbahl/Buggy/BuggyBuddies/LPC Projects/nxp_lpcxpresso_11c24_board_lib/inc" -I"/Users/vivaanbahl/Buggy/BuggyBuddies/LPC Projects/lpc_chip_11cxx_lib/inc" -O0 -fno-common -g3 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -mcpu=cortex-m0 -mthumb -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

source/ff.o: /Users/vivaanbahl/Buggy/BuggyBuddies/LPC\ Libraries/ff13/source/ff.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -DDEBUG -D__CODE_RED -DCORE_M0 -D__USE_LPCOPEN -D__LPC11XX__ -D__REDLIB__ -I"/Users/vivaanbahl/Buggy/BuggyBuddies/LPC Projects/fatfs_filesys/inc" -I"/Users/vivaanbahl/Buggy/BuggyBuddies/LPC Projects/nxp_lpcxpresso_11c24_board_lib/inc" -I"/Users/vivaanbahl/Buggy/BuggyBuddies/LPC Projects/lpc_chip_11cxx_lib/inc" -O0 -fno-common -g3 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -mcpu=cortex-m0 -mthumb -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

source/ffsystem.o: /Users/vivaanbahl/Buggy/BuggyBuddies/LPC\ Libraries/ff13/source/ffsystem.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -DDEBUG -D__CODE_RED -DCORE_M0 -D__USE_LPCOPEN -D__LPC11XX__ -D__REDLIB__ -I"/Users/vivaanbahl/Buggy/BuggyBuddies/LPC Projects/fatfs_filesys/inc" -I"/Users/vivaanbahl/Buggy/BuggyBuddies/LPC Projects/nxp_lpcxpresso_11c24_board_lib/inc" -I"/Users/vivaanbahl/Buggy/BuggyBuddies/LPC Projects/lpc_chip_11cxx_lib/inc" -O0 -fno-common -g3 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -mcpu=cortex-m0 -mthumb -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

source/ffunicode.o: /Users/vivaanbahl/Buggy/BuggyBuddies/LPC\ Libraries/ff13/source/ffunicode.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -DDEBUG -D__CODE_RED -DCORE_M0 -D__USE_LPCOPEN -D__LPC11XX__ -D__REDLIB__ -I"/Users/vivaanbahl/Buggy/BuggyBuddies/LPC Projects/fatfs_filesys/inc" -I"/Users/vivaanbahl/Buggy/BuggyBuddies/LPC Projects/nxp_lpcxpresso_11c24_board_lib/inc" -I"/Users/vivaanbahl/Buggy/BuggyBuddies/LPC Projects/lpc_chip_11cxx_lib/inc" -O0 -fno-common -g3 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -mcpu=cortex-m0 -mthumb -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


