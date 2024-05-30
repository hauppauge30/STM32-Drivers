################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Inc/rcc/rcc.c 

OBJS += \
./Inc/rcc/rcc.o 

C_DEPS += \
./Inc/rcc/rcc.d 


# Each subdirectory must supply rules for building sources it contributes
Inc/rcc/%.o Inc/rcc/%.su Inc/rcc/%.cyclo: ../Inc/rcc/%.c Inc/rcc/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F4 -DSTM32F446RETx -c -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Inc-2f-rcc

clean-Inc-2f-rcc:
	-$(RM) ./Inc/rcc/rcc.cyclo ./Inc/rcc/rcc.d ./Inc/rcc/rcc.o ./Inc/rcc/rcc.su

.PHONY: clean-Inc-2f-rcc

