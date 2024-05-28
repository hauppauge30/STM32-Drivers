################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Inc/gpio/gpio_drv.c 

OBJS += \
./Inc/gpio/gpio_drv.o 

C_DEPS += \
./Inc/gpio/gpio_drv.d 


# Each subdirectory must supply rules for building sources it contributes
Inc/gpio/%.o Inc/gpio/%.su Inc/gpio/%.cyclo: ../Inc/gpio/%.c Inc/gpio/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F4 -DSTM32F446RETx -c -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Inc-2f-gpio

clean-Inc-2f-gpio:
	-$(RM) ./Inc/gpio/gpio_drv.cyclo ./Inc/gpio/gpio_drv.d ./Inc/gpio/gpio_drv.o ./Inc/gpio/gpio_drv.su

.PHONY: clean-Inc-2f-gpio

