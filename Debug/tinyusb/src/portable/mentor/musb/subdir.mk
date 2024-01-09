################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../tinyusb/src/portable/mentor/musb/dcd_musb.c \
../tinyusb/src/portable/mentor/musb/hcd_musb.c 

OBJS += \
./tinyusb/src/portable/mentor/musb/dcd_musb.o \
./tinyusb/src/portable/mentor/musb/hcd_musb.o 

C_DEPS += \
./tinyusb/src/portable/mentor/musb/dcd_musb.d \
./tinyusb/src/portable/mentor/musb/hcd_musb.d 


# Each subdirectory must supply rules for building sources it contributes
tinyusb/src/portable/mentor/musb/%.o tinyusb/src/portable/mentor/musb/%.su tinyusb/src/portable/mentor/musb/%.cyclo: ../tinyusb/src/portable/mentor/musb/%.c tinyusb/src/portable/mentor/musb/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F412Zx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"/home/laurens/STM32CubeIDE/workspace_1.13.2/STM-Midi-Firmware/tinyusb/src" -I"/home/laurens/STM32CubeIDE/workspace_1.13.2/STM-Tiny/tinyusb/hw" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-tinyusb-2f-src-2f-portable-2f-mentor-2f-musb

clean-tinyusb-2f-src-2f-portable-2f-mentor-2f-musb:
	-$(RM) ./tinyusb/src/portable/mentor/musb/dcd_musb.cyclo ./tinyusb/src/portable/mentor/musb/dcd_musb.d ./tinyusb/src/portable/mentor/musb/dcd_musb.o ./tinyusb/src/portable/mentor/musb/dcd_musb.su ./tinyusb/src/portable/mentor/musb/hcd_musb.cyclo ./tinyusb/src/portable/mentor/musb/hcd_musb.d ./tinyusb/src/portable/mentor/musb/hcd_musb.o ./tinyusb/src/portable/mentor/musb/hcd_musb.su

.PHONY: clean-tinyusb-2f-src-2f-portable-2f-mentor-2f-musb

