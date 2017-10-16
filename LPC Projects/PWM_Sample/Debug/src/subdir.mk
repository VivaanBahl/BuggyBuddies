################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/PWM_Sample.c \
../src/cr_startup_lpc11xx.c \
../src/crp.c \
../src/steering_driver.c \
../src/sysinit.c 

OBJS += \
./src/PWM_Sample.o \
./src/cr_startup_lpc11xx.o \
./src/crp.o \
./src/steering_driver.o \
./src/sysinit.o 

C_DEPS += \
./src/PWM_Sample.d \
./src/cr_startup_lpc11xx.d \
./src/crp.d \
./src/steering_driver.d \
./src/sysinit.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -DDEBUG -D__CODE_RED -DCORE_M0 -D__USE_LPCOPEN -D__LPC11XX__ -D__REDLIB__ -I"/Users/vivaanbahl/Buggy/BuggyBuddies/LPC Projects/nxp_lpcxpresso_11c24_board_lib/inc" -I"/Users/vivaanbahl/Buggy/BuggyBuddies/LPC Projects/lpc_chip_11cxx_lib/inc" -O0 -fno-common -g3 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -mcpu=cortex-m0 -mthumb -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


