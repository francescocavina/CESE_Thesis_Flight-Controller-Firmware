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
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Templates

clean-Templates:
	-$(RM) ./Templates/src_template.cyclo ./Templates/src_template.d ./Templates/src_template.o ./Templates/src_template.su

.PHONY: clean-Templates
