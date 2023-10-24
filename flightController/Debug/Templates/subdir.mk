################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables
C_SRCS += \
../Templates/src_template.c

OBJS += \
./Templates/src_template.o

C_DEPS += \
./Templates/src_template.d


# Each subdirectory must supply rules for building sources it contributes
Templates/%.o Templates/%.su Templates/%.cyclo: ../Templates/%.c Templates/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I../APIs/Main_Application/Inc -I../Drivers/Devices/FSA8S_FlySky_RadioControlReceiver/Inc -I../Drivers/Devices/GY87_IMU/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/Devices/LoggingSystem/Inc -I../Drivers/Devices/ESC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Templates

clean-Templates:
	-$(RM) ./Templates/src_template.cyclo ./Templates/src_template.d ./Templates/src_template.o ./Templates/src_template.su

.PHONY: clean-Templates
