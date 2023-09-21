################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables
S_SRCS += \
../Core/Startup/startup_stm32f401ccux.s

OBJS += \
./Core/Startup/startup_stm32f401ccux.o

S_DEPS += \
./Core/Startup/startup_stm32f401ccux.d


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/%.o: ../Core/Startup/%.s Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -I"/mnt/CE0A6A9D0A6A8277/Francesco/Thesis/CESE_Thesis_Flight-Controller-Firmware/flightController/Settings/Inc" -I"/mnt/CE0A6A9D0A6A8277/Francesco/Thesis/CESE_Thesis_Flight-Controller-Firmware/flightController/USB_DEVICE/App" -I"/mnt/CE0A6A9D0A6A8277/Francesco/Thesis/CESE_Thesis_Flight-Controller-Firmware/flightController/USB_DEVICE/Target" -I"/mnt/CE0A6A9D0A6A8277/Francesco/Thesis/CESE_Thesis_Flight-Controller-Firmware/flightController/Drivers/Devices/MPU-6050_IMU/Src" -I"/mnt/CE0A6A9D0A6A8277/Francesco/Thesis/CESE_Thesis_Flight-Controller-Firmware/flightController/Drivers/Devices/MPU-6050_IMU/Inc" -I"/mnt/CE0A6A9D0A6A8277/Francesco/Thesis/CESE_Thesis_Flight-Controller-Firmware/flightController/Drivers/Devices/FS-A8S_FlySky_RadioControlReceiver/Inc" -I"/mnt/CE0A6A9D0A6A8277/Francesco/Thesis/CESE_Thesis_Flight-Controller-Firmware/flightController/Drivers/STM32F4xx_HAL_Driver/Inc" -I"/mnt/CE0A6A9D0A6A8277/Francesco/Thesis/CESE_Thesis_Flight-Controller-Firmware/flightController/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"/mnt/CE0A6A9D0A6A8277/Francesco/Thesis/CESE_Thesis_Flight-Controller-Firmware/flightController/Core/Inc" -I"/mnt/CE0A6A9D0A6A8277/Francesco/Thesis/CESE_Thesis_Flight-Controller-Firmware/flightController/Drivers/STM32F4xx_HAL_Driver/Src" -I"/mnt/CE0A6A9D0A6A8277/Francesco/Thesis/CESE_Thesis_Flight-Controller-Firmware/flightController/Drivers/CMSIS/Include" -I"/mnt/CE0A6A9D0A6A8277/Francesco/Thesis/CESE_Thesis_Flight-Controller-Firmware/flightController/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"/mnt/CE0A6A9D0A6A8277/Francesco/Thesis/CESE_Thesis_Flight-Controller-Firmware/flightController/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"/mnt/CE0A6A9D0A6A8277/Francesco/Thesis/CESE_Thesis_Flight-Controller-Firmware/flightController/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Core-2f-Startup

clean-Core-2f-Startup:
	-$(RM) ./Core/Startup/startup_stm32f401ccux.d ./Core/Startup/startup_stm32f401ccux.o

.PHONY: clean-Core-2f-Startup
