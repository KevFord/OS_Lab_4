################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Trace/trcInternalBuffer.c \
../Core/Trace/trcKernelPort.c \
../Core/Trace/trcSnapshotRecorder.c \
../Core/Trace/trcStreamingRecorder.c 

OBJS += \
./Core/Trace/trcInternalBuffer.o \
./Core/Trace/trcKernelPort.o \
./Core/Trace/trcSnapshotRecorder.o \
./Core/Trace/trcStreamingRecorder.o 

C_DEPS += \
./Core/Trace/trcInternalBuffer.d \
./Core/Trace/trcKernelPort.d \
./Core/Trace/trcSnapshotRecorder.d \
./Core/Trace/trcStreamingRecorder.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Trace/trcInternalBuffer.o: ../Core/Trace/trcInternalBuffer.c Core/Trace/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32WB55xx -c -I../Core/Inc -I"C:/Users/Kevin/STM32CubeIDE/encipsdatorer/OS_Lab_4/Core/Trace/extras/SDK/include" -I"C:/Users/Kevin/STM32CubeIDE/encipsdatorer/OS_Lab_4/Core/Trace/extras" -I"C:/Users/Kevin/STM32CubeIDE/encipsdatorer/OS_Lab_4/Core/Trace/config" -I"C:/Users/Kevin/STM32CubeIDE/encipsdatorer/OS_Lab_4/Core/Trace/include" -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Trace/trcInternalBuffer.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Trace/trcKernelPort.o: ../Core/Trace/trcKernelPort.c Core/Trace/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32WB55xx -c -I../Core/Inc -I"C:/Users/Kevin/STM32CubeIDE/encipsdatorer/OS_Lab_4/Core/Trace/extras/SDK/include" -I"C:/Users/Kevin/STM32CubeIDE/encipsdatorer/OS_Lab_4/Core/Trace/extras" -I"C:/Users/Kevin/STM32CubeIDE/encipsdatorer/OS_Lab_4/Core/Trace/config" -I"C:/Users/Kevin/STM32CubeIDE/encipsdatorer/OS_Lab_4/Core/Trace/include" -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Trace/trcKernelPort.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Trace/trcSnapshotRecorder.o: ../Core/Trace/trcSnapshotRecorder.c Core/Trace/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32WB55xx -c -I../Core/Inc -I"C:/Users/Kevin/STM32CubeIDE/encipsdatorer/OS_Lab_4/Core/Trace/extras/SDK/include" -I"C:/Users/Kevin/STM32CubeIDE/encipsdatorer/OS_Lab_4/Core/Trace/extras" -I"C:/Users/Kevin/STM32CubeIDE/encipsdatorer/OS_Lab_4/Core/Trace/config" -I"C:/Users/Kevin/STM32CubeIDE/encipsdatorer/OS_Lab_4/Core/Trace/include" -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Trace/trcSnapshotRecorder.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Trace/trcStreamingRecorder.o: ../Core/Trace/trcStreamingRecorder.c Core/Trace/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32WB55xx -c -I../Core/Inc -I"C:/Users/Kevin/STM32CubeIDE/encipsdatorer/OS_Lab_4/Core/Trace/extras/SDK/include" -I"C:/Users/Kevin/STM32CubeIDE/encipsdatorer/OS_Lab_4/Core/Trace/extras" -I"C:/Users/Kevin/STM32CubeIDE/encipsdatorer/OS_Lab_4/Core/Trace/config" -I"C:/Users/Kevin/STM32CubeIDE/encipsdatorer/OS_Lab_4/Core/Trace/include" -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Trace/trcStreamingRecorder.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

