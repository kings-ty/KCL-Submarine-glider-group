################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/app/app.c \
../Core/Src/app/demo.c \
../Core/Src/app/mission.c \
../Core/Src/app/system_check.c 

OBJS += \
./Core/Src/app/app.o \
./Core/Src/app/demo.o \
./Core/Src/app/mission.o \
./Core/Src/app/system_check.o 

C_DEPS += \
./Core/Src/app/app.d \
./Core/Src/app/demo.d \
./Core/Src/app/mission.d \
./Core/Src/app/system_check.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/app/%.o Core/Src/app/%.su Core/Src/app/%.cyclo: ../Core/Src/app/%.c Core/Src/app/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-app

clean-Core-2f-Src-2f-app:
	-$(RM) ./Core/Src/app/app.cyclo ./Core/Src/app/app.d ./Core/Src/app/app.o ./Core/Src/app/app.su ./Core/Src/app/demo.cyclo ./Core/Src/app/demo.d ./Core/Src/app/demo.o ./Core/Src/app/demo.su ./Core/Src/app/mission.cyclo ./Core/Src/app/mission.d ./Core/Src/app/mission.o ./Core/Src/app/mission.su ./Core/Src/app/system_check.cyclo ./Core/Src/app/system_check.d ./Core/Src/app/system_check.o ./Core/Src/app/system_check.su

.PHONY: clean-Core-2f-Src-2f-app

