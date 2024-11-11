################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Drivers/App/imager.cpp 

OBJS += \
./Drivers/App/imager.o 

CPP_DEPS += \
./Drivers/App/imager.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/App/%.o Drivers/App/%.su Drivers/App/%.cyclo: ../Drivers/App/%.cpp Drivers/App/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m0plus -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L053xx -c -I../Core/Inc -I../Drivers/App -I../Drivers/STM32L0xx_HAL_Driver/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L0xx/Include -I../Drivers/CMSIS/Include -I../Drivers/Camera_driver -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-App

clean-Drivers-2f-App:
	-$(RM) ./Drivers/App/imager.cyclo ./Drivers/App/imager.d ./Drivers/App/imager.o ./Drivers/App/imager.su

.PHONY: clean-Drivers-2f-App

