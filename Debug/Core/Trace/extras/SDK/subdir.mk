################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Trace/extras/SDK/trcSDK.c 

OBJS += \
./Core/Trace/extras/SDK/trcSDK.o 

C_DEPS += \
./Core/Trace/extras/SDK/trcSDK.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Trace/extras/SDK/trcSDK.o: ../Core/Trace/extras/SDK/trcSDK.c Core/Trace/extras/SDK/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32WB55xx -c -I../Core/Inc -I"C:/Users/Kevin/STM32CubeIDE/encipsdatorer/OS_Lab_4/Core/Trace/extras/SDK/include" -I"C:/Users/Kevin/STM32CubeIDE/encipsdatorer/OS_Lab_4/Core/Trace/extras" -I"C:/Users/Kevin/STM32CubeIDE/encipsdatorer/OS_Lab_4/Core/Trace/config" -I"C:/Users/Kevin/STM32CubeIDE/encipsdatorer/OS_Lab_4/Core/Trace/include" -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Trace/extras/SDK/trcSDK.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

