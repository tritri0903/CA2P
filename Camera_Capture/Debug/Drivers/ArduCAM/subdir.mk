################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/ArduCAM/ArduCAM.c 

C_DEPS += \
./Drivers/ArduCAM/ArduCAM.d 

OBJS += \
./Drivers/ArduCAM/ArduCAM.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/ArduCAM/%.o Drivers/ArduCAM/%.su Drivers/ArduCAM/%.cyclo: ../Drivers/ArduCAM/%.c Drivers/ArduCAM/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L053xx -c -I../Core/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L0xx/Include -I../Drivers/ArduCAM -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I"C:/Users/Tristan/Documents/Github/CA2P/Camera_Capture/Drivers/SD_CARD" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-ArduCAM

clean-Drivers-2f-ArduCAM:
	-$(RM) ./Drivers/ArduCAM/ArduCAM.cyclo ./Drivers/ArduCAM/ArduCAM.d ./Drivers/ArduCAM/ArduCAM.o ./Drivers/ArduCAM/ArduCAM.su

.PHONY: clean-Drivers-2f-ArduCAM

