################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables
C_SRCS += \
../APIs/Main_Application/Src/main_app.c

OBJS += \
./APIs/Main_Application/Src/main_app.o

C_DEPS += \
./APIs/Main_Application/Src/main_app.d


# Each subdirectory must supply rules for building sources it contributes
APIs/Main_Application/Src/%.o APIs/Main_Application/Src/%.su APIs/Main_Application/Src/%.cyclo: ../APIs/Main_Application/Src/%.c APIs/Main_Application/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I../Drivers/Devices/FSA8S_FlySky_RadioControlReceiver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/Devices/LoggingSystem/Inc -I../APIs/Main_Application/Inc -I"/mnt/CE0A6A9D0A6A8277/Francesco/Thesis/CESE_Thesis_Flight-Controller-Firmware/flightController/APIs/Main_Application" -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-APIs-2f-Main_Application-2f-Src

clean-APIs-2f-Main_Application-2f-Src:
	-$(RM) ./APIs/Main_Application/Src/main_app.cyclo ./APIs/Main_Application/Src/main_app.d ./APIs/Main_Application/Src/main_app.o ./APIs/Main_Application/Src/main_app.su

.PHONY: clean-APIs-2f-Main_Application-2f-Src
