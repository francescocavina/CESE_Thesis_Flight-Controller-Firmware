################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables
C_SRCS += \
../Drivers/Devices/FS-A8S_Flysky_RadioControlReceiver/Src/FS-A8S_driver_HWI.c \
../Drivers/Devices/FS-A8S_Flysky_RadioControlReceiver/Src/FS-A8S_driver_UAI.c \
../Drivers/Devices/FS-A8S_Flysky_RadioControlReceiver/Src/FS-A8S_driver_demo.c

OBJS += \
./Drivers/Devices/FS-A8S_Flysky_RadioControlReceiver/Src/FS-A8S_driver_HWI.o \
./Drivers/Devices/FS-A8S_Flysky_RadioControlReceiver/Src/FS-A8S_driver_UAI.o \
./Drivers/Devices/FS-A8S_Flysky_RadioControlReceiver/Src/FS-A8S_driver_demo.o

C_DEPS += \
./Drivers/Devices/FS-A8S_Flysky_RadioControlReceiver/Src/FS-A8S_driver_HWI.d \
./Drivers/Devices/FS-A8S_Flysky_RadioControlReceiver/Src/FS-A8S_driver_UAI.d \
./Drivers/Devices/FS-A8S_Flysky_RadioControlReceiver/Src/FS-A8S_driver_demo.d


# Each subdirectory must supply rules for building sources it contributes
Drivers/Devices/FS-A8S_Flysky_RadioControlReceiver/Src/%.o Drivers/Devices/FS-A8S_Flysky_RadioControlReceiver/Src/%.su Drivers/Devices/FS-A8S_Flysky_RadioControlReceiver/Src/%.cyclo: ../Drivers/Devices/FS-A8S_Flysky_RadioControlReceiver/Src/%.c Drivers/Devices/FS-A8S_Flysky_RadioControlReceiver/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/mnt/CE0A6A9D0A6A8277/Francesco/Thesis/CESE_Thesis_Flight-Controller-Firmware/flightController/Drivers/Devices/FS-A8S_Flysky_RadioControlReceiver/Src" -I"/mnt/CE0A6A9D0A6A8277/Francesco/Thesis/CESE_Thesis_Flight-Controller-Firmware/flightController/Drivers/Devices/FS-A8S_Flysky_RadioControlReceiver/Inc" -I"/mnt/CE0A6A9D0A6A8277/Francesco/Thesis/CESE_Thesis_Flight-Controller-Firmware/flightController/Settings/Inc" -I"/mnt/CE0A6A9D0A6A8277/Francesco/Thesis/CESE_Thesis_Flight-Controller-Firmware/flightController/USB_DEVICE/App" -I"/mnt/CE0A6A9D0A6A8277/Francesco/Thesis/CESE_Thesis_Flight-Controller-Firmware/flightController/USB_DEVICE/Target" -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I"/mnt/CE0A6A9D0A6A8277/Francesco/Thesis/CESE_Thesis_Flight-Controller-Firmware/flightController/Drivers/Devices/MPU-6050_IMU/Src" -I"/mnt/CE0A6A9D0A6A8277/Francesco/Thesis/CESE_Thesis_Flight-Controller-Firmware/flightController/Drivers/Devices/MPU-6050_IMU/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Devices-2f-FS-2d-A8S_Flysky_RadioControlReceiver-2f-Src

clean-Drivers-2f-Devices-2f-FS-2d-A8S_Flysky_RadioControlReceiver-2f-Src:
	-$(RM) ./Drivers/Devices/FS-A8S_Flysky_RadioControlReceiver/Src/FS-A8S_driver_HWI.cyclo ./Drivers/Devices/FS-A8S_Flysky_RadioControlReceiver/Src/FS-A8S_driver_HWI.d ./Drivers/Devices/FS-A8S_Flysky_RadioControlReceiver/Src/FS-A8S_driver_HWI.o ./Drivers/Devices/FS-A8S_Flysky_RadioControlReceiver/Src/FS-A8S_driver_HWI.su ./Drivers/Devices/FS-A8S_Flysky_RadioControlReceiver/Src/FS-A8S_driver_UAI.cyclo ./Drivers/Devices/FS-A8S_Flysky_RadioControlReceiver/Src/FS-A8S_driver_UAI.d ./Drivers/Devices/FS-A8S_Flysky_RadioControlReceiver/Src/FS-A8S_driver_UAI.o ./Drivers/Devices/FS-A8S_Flysky_RadioControlReceiver/Src/FS-A8S_driver_UAI.su ./Drivers/Devices/FS-A8S_Flysky_RadioControlReceiver/Src/FS-A8S_driver_demo.cyclo ./Drivers/Devices/FS-A8S_Flysky_RadioControlReceiver/Src/FS-A8S_driver_demo.d ./Drivers/Devices/FS-A8S_Flysky_RadioControlReceiver/Src/FS-A8S_driver_demo.o ./Drivers/Devices/FS-A8S_Flysky_RadioControlReceiver/Src/FS-A8S_driver_demo.su

.PHONY: clean-Drivers-2f-Devices-2f-FS-2d-A8S_Flysky_RadioControlReceiver-2f-Src
