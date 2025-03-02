################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables
C_SRCS += \
../Drivers/Devices/GY87_IMU/Src/MPU6050_driver_HWI.c \
../Drivers/Devices/GY87_IMU/Src/MPU6050_driver_UAI.c

OBJS += \
./Drivers/Devices/GY87_IMU/Src/MPU6050_driver_HWI.o \
./Drivers/Devices/GY87_IMU/Src/MPU6050_driver_UAI.o

C_DEPS += \
./Drivers/Devices/GY87_IMU/Src/MPU6050_driver_HWI.d \
./Drivers/Devices/GY87_IMU/Src/MPU6050_driver_UAI.d


# Each subdirectory must supply rules for building sources it contributes
Drivers/Devices/GY87_IMU/Src/%.o Drivers/Devices/GY87_IMU/Src/%.su Drivers/Devices/GY87_IMU/Src/%.cyclo: ../Drivers/Devices/GY87_IMU/Src/%.c Drivers/Devices/GY87_IMU/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I"D:/Francesco/Thesis/CESE_Thesis_Flight-Controller-Firmware/flightController/flightController/Drivers/Devices/LoggingSystem/Inc" -I"D:/Francesco/Thesis/CESE_Thesis_Flight-Controller-Firmware/flightController/flightController/Drivers/Devices/GY87_IMU/Inc" -I"D:/Francesco/Thesis/CESE_Thesis_Flight-Controller-Firmware/flightController/flightController/Drivers/Devices/FSA8S_FlySky_RadioControlReceiver/Inc" -I"D:/Francesco/Thesis/CESE_Thesis_Flight-Controller-Firmware/flightController/flightController/Drivers/Devices/ESC/Inc" -I"D:/Francesco/Thesis/CESE_Thesis_Flight-Controller-Firmware/flightController/flightController/APIs/Main_Application/Inc" -I"D:/Francesco/Thesis/CESE_Thesis_Flight-Controller-Firmware/flightController/flightController/Settings/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Devices-2f-GY87_IMU-2f-Src

clean-Drivers-2f-Devices-2f-GY87_IMU-2f-Src:
	-$(RM) ./Drivers/Devices/GY87_IMU/Src/MPU6050_driver_HWI.cyclo ./Drivers/Devices/GY87_IMU/Src/MPU6050_driver_HWI.d ./Drivers/Devices/GY87_IMU/Src/MPU6050_driver_HWI.o ./Drivers/Devices/GY87_IMU/Src/MPU6050_driver_HWI.su ./Drivers/Devices/GY87_IMU/Src/MPU6050_driver_UAI.cyclo ./Drivers/Devices/GY87_IMU/Src/MPU6050_driver_UAI.d ./Drivers/Devices/GY87_IMU/Src/MPU6050_driver_UAI.o ./Drivers/Devices/GY87_IMU/Src/MPU6050_driver_UAI.su

.PHONY: clean-Drivers-2f-Devices-2f-GY87_IMU-2f-Src
