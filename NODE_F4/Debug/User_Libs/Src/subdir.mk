################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../User_Libs/Src/DS3231.c \
../User_Libs/Src/Fonts.c \
../User_Libs/Src/Keypad.c \
../User_Libs/Src/Lora.c \
../User_Libs/Src/TFT.c \
../User_Libs/Src/W25Q.c 

OBJS += \
./User_Libs/Src/DS3231.o \
./User_Libs/Src/Fonts.o \
./User_Libs/Src/Keypad.o \
./User_Libs/Src/Lora.o \
./User_Libs/Src/TFT.o \
./User_Libs/Src/W25Q.o 

C_DEPS += \
./User_Libs/Src/DS3231.d \
./User_Libs/Src/Fonts.d \
./User_Libs/Src/Keypad.d \
./User_Libs/Src/Lora.d \
./User_Libs/Src/TFT.d \
./User_Libs/Src/W25Q.d 


# Each subdirectory must supply rules for building sources it contributes
User_Libs/Src/%.o User_Libs/Src/%.su: ../User_Libs/Src/%.c User_Libs/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../User_Libs/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-User_Libs-2f-Src

clean-User_Libs-2f-Src:
	-$(RM) ./User_Libs/Src/DS3231.d ./User_Libs/Src/DS3231.o ./User_Libs/Src/DS3231.su ./User_Libs/Src/Fonts.d ./User_Libs/Src/Fonts.o ./User_Libs/Src/Fonts.su ./User_Libs/Src/Keypad.d ./User_Libs/Src/Keypad.o ./User_Libs/Src/Keypad.su ./User_Libs/Src/Lora.d ./User_Libs/Src/Lora.o ./User_Libs/Src/Lora.su ./User_Libs/Src/TFT.d ./User_Libs/Src/TFT.o ./User_Libs/Src/TFT.su ./User_Libs/Src/W25Q.d ./User_Libs/Src/W25Q.o ./User_Libs/Src/W25Q.su

.PHONY: clean-User_Libs-2f-Src

