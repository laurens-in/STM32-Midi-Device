################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../tinyusb/src/portable/ehci/ehci.c 

OBJS += \
./tinyusb/src/portable/ehci/ehci.o 

C_DEPS += \
./tinyusb/src/portable/ehci/ehci.d 


# Each subdirectory must supply rules for building sources it contributes
tinyusb/src/portable/ehci/%.o tinyusb/src/portable/ehci/%.su tinyusb/src/portable/ehci/%.cyclo: ../tinyusb/src/portable/ehci/%.c tinyusb/src/portable/ehci/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F412Zx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"/home/laurens/STM32CubeIDE/workspace_1.13.2/STM-Midi-Firmware/tinyusb/src" -I"/home/laurens/STM32CubeIDE/workspace_1.13.2/STM-Tiny/tinyusb/hw" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-tinyusb-2f-src-2f-portable-2f-ehci

clean-tinyusb-2f-src-2f-portable-2f-ehci:
	-$(RM) ./tinyusb/src/portable/ehci/ehci.cyclo ./tinyusb/src/portable/ehci/ehci.d ./tinyusb/src/portable/ehci/ehci.o ./tinyusb/src/portable/ehci/ehci.su

.PHONY: clean-tinyusb-2f-src-2f-portable-2f-ehci

