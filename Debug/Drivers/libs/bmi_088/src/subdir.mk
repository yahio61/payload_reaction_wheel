################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/libs/bmi_088/src/example.c 

OBJS += \
./Drivers/libs/bmi_088/src/example.o 

C_DEPS += \
./Drivers/libs/bmi_088/src/example.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/libs/bmi_088/src/%.o Drivers/libs/bmi_088/src/%.su Drivers/libs/bmi_088/src/%.cyclo: ../Drivers/libs/bmi_088/src/%.c Drivers/libs/bmi_088/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/yahya/STM32CubeIDE/workspace_1.19.0/payload_reaction_wheel/Drivers/libs/bmi_088/bmi088" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-libs-2f-bmi_088-2f-src

clean-Drivers-2f-libs-2f-bmi_088-2f-src:
	-$(RM) ./Drivers/libs/bmi_088/src/example.cyclo ./Drivers/libs/bmi_088/src/example.d ./Drivers/libs/bmi_088/src/example.o ./Drivers/libs/bmi_088/src/example.su

.PHONY: clean-Drivers-2f-libs-2f-bmi_088-2f-src

