################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables
C_SRCS += \
../Drivers/Devices/FSA8S_FlySky_RadioControlReceiver/Src/FSA8S_driver_HWI.c \
../Drivers/Devices/FSA8S_FlySky_RadioControlReceiver/Src/FSA8S_driver_UAI.c \
../Drivers/Devices/FSA8S_FlySky_RadioControlReceiver/Src/FSA8S_driver_demo.c

OBJS += \
./Drivers/Devices/FSA8S_FlySky_RadioControlReceiver/Src/FSA8S_driver_HWI.o \
./Drivers/Devices/FSA8S_FlySky_RadioControlReceiver/Src/FSA8S_driver_UAI.o \
./Drivers/Devices/FSA8S_FlySky_RadioControlReceiver/Src/FSA8S_driver_demo.o

C_DEPS += \
./Drivers/Devices/FSA8S_FlySky_RadioControlReceiver/Src/FSA8S_driver_HWI.d \
./Drivers/Devices/FSA8S_FlySky_RadioControlReceiver/Src/FSA8S_driver_UAI.d \
./Drivers/Devices/FSA8S_FlySky_RadioControlReceiver/Src/FSA8S_driver_demo.d


# Each subdirectory must supply rules for building sources it contributes
Drivers/Devices/FSA8S_FlySky_RadioControlReceiver/Src/%.o Drivers/Devices/FSA8S_FlySky_RadioControlReceiver/Src/%.su Drivers/Devices/FSA8S_FlySky_RadioControlReceiver/Src/%.cyclo: ../Drivers/Devices/FSA8S_FlySky_RadioControlReceiver/Src/%.c Drivers/Devices/FSA8S_FlySky_RadioControlReceiver/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I../Drivers/Devices/FSA8S_FlySky_RadioControlReceiver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/Devices/LoggingSystem/Inc -I../APIs/Main_Application/Inc -I"/mnt/CE0A6A9D0A6A8277/Francesco/Thesis/CESE_Thesis_Flight-Controller-Firmware/flightController/APIs/Main_Application" -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Devices-2f-FSA8S_FlySky_RadioControlReceiver-2f-Src

clean-Drivers-2f-Devices-2f-FSA8S_FlySky_RadioControlReceiver-2f-Src:
	-$(RM) ./Drivers/Devices/FSA8S_FlySky_RadioControlReceiver/Src/FSA8S_driver_HWI.cyclo ./Drivers/Devices/FSA8S_FlySky_RadioControlReceiver/Src/FSA8S_driver_HWI.d ./Drivers/Devices/FSA8S_FlySky_RadioControlReceiver/Src/FSA8S_driver_HWI.o ./Drivers/Devices/FSA8S_FlySky_RadioControlReceiver/Src/FSA8S_driver_HWI.su ./Drivers/Devices/FSA8S_FlySky_RadioControlReceiver/Src/FSA8S_driver_UAI.cyclo ./Drivers/Devices/FSA8S_FlySky_RadioControlReceiver/Src/FSA8S_driver_UAI.d ./Drivers/Devices/FSA8S_FlySky_RadioControlReceiver/Src/FSA8S_driver_UAI.o ./Drivers/Devices/FSA8S_FlySky_RadioControlReceiver/Src/FSA8S_driver_UAI.su ./Drivers/Devices/FSA8S_FlySky_RadioControlReceiver/Src/FSA8S_driver_demo.cyclo ./Drivers/Devices/FSA8S_FlySky_RadioControlReceiver/Src/FSA8S_driver_demo.d ./Drivers/Devices/FSA8S_FlySky_RadioControlReceiver/Src/FSA8S_driver_demo.o ./Drivers/Devices/FSA8S_FlySky_RadioControlReceiver/Src/FSA8S_driver_demo.su

.PHONY: clean-Drivers-2f-Devices-2f-FSA8S_FlySky_RadioControlReceiver-2f-Src
