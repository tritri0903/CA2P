################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Camera_driver/OV2640.cpp \
../Camera_driver/OV2640_regs.cpp \
../Camera_driver/OV5642.cpp \
../Camera_driver/OV5642_regs.cpp \
../Camera_driver/SPI_camera.cpp 

OBJS += \
./Camera_driver/OV2640.o \
./Camera_driver/OV2640_regs.o \
./Camera_driver/OV5642.o \
./Camera_driver/OV5642_regs.o \
./Camera_driver/SPI_camera.o 

CPP_DEPS += \
./Camera_driver/OV2640.d \
./Camera_driver/OV2640_regs.d \
./Camera_driver/OV5642.d \
./Camera_driver/OV5642_regs.d \
./Camera_driver/SPI_camera.d 


# Each subdirectory must supply rules for building sources it contributes
Camera_driver/%.o Camera_driver/%.su Camera_driver/%.cyclo: ../Camera_driver/%.cpp Camera_driver/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m0plus -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L053xx -c -I../Core/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L0xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/trist/OneDrive - Enseignement de la Province de Liège/M2 Q1/CA2P/Hardware/Software/Camera_Capture/Camera_driver" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Camera_driver

clean-Camera_driver:
	-$(RM) ./Camera_driver/OV2640.cyclo ./Camera_driver/OV2640.d ./Camera_driver/OV2640.o ./Camera_driver/OV2640.su ./Camera_driver/OV2640_regs.cyclo ./Camera_driver/OV2640_regs.d ./Camera_driver/OV2640_regs.o ./Camera_driver/OV2640_regs.su ./Camera_driver/OV5642.cyclo ./Camera_driver/OV5642.d ./Camera_driver/OV5642.o ./Camera_driver/OV5642.su ./Camera_driver/OV5642_regs.cyclo ./Camera_driver/OV5642_regs.d ./Camera_driver/OV5642_regs.o ./Camera_driver/OV5642_regs.su ./Camera_driver/SPI_camera.cyclo ./Camera_driver/SPI_camera.d ./Camera_driver/SPI_camera.o ./Camera_driver/SPI_camera.su

.PHONY: clean-Camera_driver

