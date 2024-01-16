################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables
C_SRCS += \
../Drivers/Devices/LoggingSystem/Src/LoggingSystem_HWI.c \
../Drivers/Devices/LoggingSystem/Src/LoggingSystem_UAI.c

OBJS += \
./Drivers/Devices/LoggingSystem/Src/LoggingSystem_HWI.o \
./Drivers/Devices/LoggingSystem/Src/LoggingSystem_UAI.o

C_DEPS += \
./Drivers/Devices/LoggingSystem/Src/LoggingSystem_HWI.d \
./Drivers/Devices/LoggingSystem/Src/LoggingSystem_UAI.d


# Each subdirectory must supply rules for building sources it contributes
Drivers/Devices/LoggingSystem/Src/%.o Drivers/Devices/LoggingSystem/Src/%.su Drivers/Devices/LoggingSystem/Src/%.cyclo: ../Drivers/Devices/LoggingSystem/Src/%.c Drivers/Devices/LoggingSystem/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I"/mnt/CE0A6A9D0A6A8277/Francesco/Thesis/CESE_Thesis_Flight-Controller-Firmware/flightController/Drivers/Devices/PowerOnOff/Inc" -I"/mnt/CE0A6A9D0A6A8277/Francesco/Thesis/CESE_Thesis_Flight-Controller-Firmware/flightController/Drivers/Devices/LoggingSystem/Inc" -I"/mnt/CE0A6A9D0A6A8277/Francesco/Thesis/CESE_Thesis_Flight-Controller-Firmware/flightController/Drivers/Devices/GY87_IMU/Inc" -I"/mnt/CE0A6A9D0A6A8277/Francesco/Thesis/CESE_Thesis_Flight-Controller-Firmware/flightController/Drivers/Devices/FSA8S_FlySky_RadioControlReceiver/Inc" -I"/mnt/CE0A6A9D0A6A8277/Francesco/Thesis/CESE_Thesis_Flight-Controller-Firmware/flightController/Drivers/Devices/ESC/Inc" -I"/mnt/CE0A6A9D0A6A8277/Francesco/Thesis/CESE_Thesis_Flight-Controller-Firmware/flightController/APIs/Main_Application/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Devices-2f-LoggingSystem-2f-Src

clean-Drivers-2f-Devices-2f-LoggingSystem-2f-Src:
	-$(RM) ./Drivers/Devices/LoggingSystem/Src/LoggingSystem_HWI.cyclo ./Drivers/Devices/LoggingSystem/Src/LoggingSystem_HWI.d ./Drivers/Devices/LoggingSystem/Src/LoggingSystem_HWI.o ./Drivers/Devices/LoggingSystem/Src/LoggingSystem_HWI.su ./Drivers/Devices/LoggingSystem/Src/LoggingSystem_UAI.cyclo ./Drivers/Devices/LoggingSystem/Src/LoggingSystem_UAI.d ./Drivers/Devices/LoggingSystem/Src/LoggingSystem_UAI.o ./Drivers/Devices/LoggingSystem/Src/LoggingSystem_UAI.su

.PHONY: clean-Drivers-2f-Devices-2f-LoggingSystem-2f-Src
