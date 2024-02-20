################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/MPU6050/I2Cdev.cpp \
../Core/MPU6050/MPU6050.cpp \
../Core/MPU6050/MPU6050_6Axis_MotionApps20.cpp 

OBJS += \
./Core/MPU6050/I2Cdev.o \
./Core/MPU6050/MPU6050.o \
./Core/MPU6050/MPU6050_6Axis_MotionApps20.o 

CPP_DEPS += \
./Core/MPU6050/I2Cdev.d \
./Core/MPU6050/MPU6050.d \
./Core/MPU6050/MPU6050_6Axis_MotionApps20.d 


# Each subdirectory must supply rules for building sources it contributes
Core/MPU6050/%.o Core/MPU6050/%.su: ../Core/MPU6050/%.cpp Core/MPU6050/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m0 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F072xB -c -I../Core/Inc -I"C:/Users/Solov/STM32CubeIDE/workspace_1.11.0/JackSparrow/Core/MPU6050" -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-MPU6050

clean-Core-2f-MPU6050:
	-$(RM) ./Core/MPU6050/I2Cdev.d ./Core/MPU6050/I2Cdev.o ./Core/MPU6050/I2Cdev.su ./Core/MPU6050/MPU6050.d ./Core/MPU6050/MPU6050.o ./Core/MPU6050/MPU6050.su ./Core/MPU6050/MPU6050_6Axis_MotionApps20.d ./Core/MPU6050/MPU6050_6Axis_MotionApps20.o ./Core/MPU6050/MPU6050_6Axis_MotionApps20.su

.PHONY: clean-Core-2f-MPU6050

