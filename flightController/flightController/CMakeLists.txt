cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME flightController)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ./Core/Src/main.c
    ./Core/Src/freertos.c
    ./Core/Src/stm32f4xx_it.c
    ./Core/Src/stm32f4xx_hal_msp.c
    ./APIs/Main_Application/Src/main_app.c
    ./APIs/ErrorLED/Src/ErrorLED.c
    ./USB_DEVICE/App/usb_device.c
    ./USB_DEVICE/App/usbd_desc.c
    ./USB_DEVICE/App/usbd_cdc_if.c
    ./USB_DEVICE/Target/usbd_conf.c
    ./Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd.c
    ./Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd_ex.c
    ./Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_usb.c
    ./Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
    ./Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c
    ./Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c
    ./Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c
    ./Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c
    ./Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
    ./Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c
    ./Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c
    ./Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c
    ./Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c
    ./Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c
    ./Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
    ./Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c
    ./Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc.c
    ./Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc_ex.c
    ./Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_adc.c
    ./Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c
    ./Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c_ex.c
    ./Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c
    ./Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c
    ./Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c
    ./Core/Src/system_stm32f4xx.c
    ./Middlewares/Third_Party/FreeRTOS/Source/croutine.c
    ./Middlewares/Third_Party/FreeRTOS/Source/event_groups.c
    ./Middlewares/Third_Party/FreeRTOS/Source/list.c
    ./Middlewares/Third_Party/FreeRTOS/Source/queue.c
    ./Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c
    ./Middlewares/Third_Party/FreeRTOS/Source/tasks.c
    ./Middlewares/Third_Party/FreeRTOS/Source/timers.c
    ./Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c
    ./Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c
    ./Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c
    ./Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
    ./Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
    ./Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c
    ./Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c
    ./Core/Src/sysmem.c
    ./Core/Src/syscalls.c
    ./startup_stm32f401xc.s
    ./Drivers/Devices/ESC/Src/ESC_UAI.c
    ./Drivers/Devices/ESC/Src/ESC_HWI.c
    ./Drivers/Devices/FSA8S_FlySky_RadioControlReceiver/Src/FSA8S_driver_UAI.c
    ./Drivers/Devices/FSA8S_FlySky_RadioControlReceiver/Src/FSA8S_driver_HWI.c
    ./Drivers/Devices/GY87_IMU/Src/MPU6050_driver_UAI.c
    ./Drivers/Devices/GY87_IMU/Src/MPU6050_driver_HWI.c
    ./Drivers/Devices/LoggingSystem/Src/LoggingSystem_UAI.c
    ./Drivers/Devices/LoggingSystem/Src/LoggingSystem_HWI.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ./Core/Inc
    ./APIs/Main_Application/Inc
    ./APIs/ErrorLED/Inc
    ./USB_DEVICE/App
    ./USB_DEVICE/Target
    ./Drivers/STM32F4xx_HAL_Driver/Inc
    ./Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
    ./Middlewares/Third_Party/FreeRTOS/Source/include
    ./Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS
    ./Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
    ./Middlewares/ST/STM32_USB_Device_Library/Core/Inc
    ./Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
    ./Drivers/CMSIS/Device/ST/STM32F4xx/Include
    ./Drivers/CMSIS/Include
    ./Drivers/Devices/ESC/Inc
    ./Drivers/Devices/FSA8S_FlySky_RadioControlReceiver/Inc
    ./Drivers/Devices/GY87_IMU/Inc
    ./Drivers/Devices/LoggingSystem/Inc
    ./Settings/Inc
)



# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    -u_printf_float
    # Add user defined libraries
)
