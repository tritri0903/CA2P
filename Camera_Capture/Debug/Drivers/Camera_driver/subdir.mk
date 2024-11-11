################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Drivers/Camera_driver/OV2640.cpp \
../Drivers/Camera_driver/OV2640_regs.cpp \
../Drivers/Camera_driver/OV5642.cpp \
../Drivers/Camera_driver/OV5642_regs.cpp \
../Drivers/Camera_driver/SPI_camera.cpp 

OBJS += \
./Drivers/Camera_driver/OV2640.o \
./Drivers/Camera_driver/OV2640_regs.o \
./Drivers/Camera_driver/OV5642.o \
./Drivers/Camera_driver/OV5642_regs.o \
./Drivers/Camera_driver/SPI_camera.o 

CPP_DEPS += \
./Drivers/Camera_driver/OV2640.d \
./Drivers/Camera_driver/OV2640_regs.d \
./Drivers/Camera_driver/OV5642.d \
./Drivers/Camera_driver/OV5642_regs.d \
./Drivers/Camera_driver/SPI_camera.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Camera_driver/%.o Drivers/Camera_driver/%.su Drivers/Camera_driver/%.cyclo: ../Drivers/Camera_driver/%.cpp Drivers/Camera_driver/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m0plus -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L053xx -c -I../Core/Inc -I../Drivers/App -I../Drivers/STM32L0xx_HAL_Driver/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L0xx/Include -I../Drivers/CMSIS/Include -I../Drivers/Camera_driver -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-Camera_driver

clean-Drivers-2f-Camera_driver:
	-$(RM) ./Drivers/Camera_driver/OV2640.cyclo ./Drivers/Camera_driver/OV2640.d ./Drivers/Camera_driver/OV2640.o ./Drivers/Camera_driver/OV2640.su ./Drivers/Camera_driver/OV2640_regs.cyclo ./Drivers/Camera_driver/OV2640_regs.d ./Drivers/Camera_driver/OV2640_regs.o ./Drivers/Camera_driver/OV2640_regs.su ./Drivers/Camera_driver/OV5642.cyclo ./Drivers/Camera_driver/OV5642.d ./Drivers/Camera_driver/OV5642.o ./Drivers/Camera_driver/OV5642.su ./Drivers/Camera_driver/OV5642_regs.cyclo ./Drivers/Camera_driver/OV5642_regs.d ./Drivers/Camera_driver/OV5642_regs.o ./Drivers/Camera_driver/OV5642_regs.su ./Drivers/Camera_driver/SPI_camera.cyclo ./Drivers/Camera_driver/SPI_camera.d ./Drivers/Camera_driver/SPI_camera.o ./Drivers/Camera_driver/SPI_camera.su

.PHONY: clean-Drivers-2f-Camera_driver

