################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/RingBuffer.c \
../Core/Src/StateMachine.c \
../Core/Src/freertos.c \
../Core/Src/main.c \
../Core/Src/sensorBMP388.c \
../Core/Src/sensorLIS2MDL.c \
../Core/Src/sensorLSM6DSO32.c \
../Core/Src/sensorShared.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_hal_timebase_tim.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/taskBMP388.c \
../Core/Src/taskLIS2MDL.c \
../Core/Src/taskLSM6DSO32.c \
../Core/Src/taskShared.c \
../Core/Src/taskStateMachine.c \
../Core/Src/taskUART.c 

OBJS += \
./Core/Src/RingBuffer.o \
./Core/Src/StateMachine.o \
./Core/Src/freertos.o \
./Core/Src/main.o \
./Core/Src/sensorBMP388.o \
./Core/Src/sensorLIS2MDL.o \
./Core/Src/sensorLSM6DSO32.o \
./Core/Src/sensorShared.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_hal_timebase_tim.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/taskBMP388.o \
./Core/Src/taskLIS2MDL.o \
./Core/Src/taskLSM6DSO32.o \
./Core/Src/taskShared.o \
./Core/Src/taskStateMachine.o \
./Core/Src/taskUART.o 

C_DEPS += \
./Core/Src/RingBuffer.d \
./Core/Src/StateMachine.d \
./Core/Src/freertos.d \
./Core/Src/main.d \
./Core/Src/sensorBMP388.d \
./Core/Src/sensorLIS2MDL.d \
./Core/Src/sensorLSM6DSO32.d \
./Core/Src/sensorShared.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_hal_timebase_tim.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/taskBMP388.d \
./Core/Src/taskLIS2MDL.d \
./Core/Src/taskLSM6DSO32.d \
./Core/Src/taskShared.d \
./Core/Src/taskStateMachine.d \
./Core/Src/taskUART.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/DSP/Include -I"../Drivers/CMSIS/DSP/PrivateInclude" -I"../Drivers/CMSIS/DSP/ComputeLibrary/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/RingBuffer.cyclo ./Core/Src/RingBuffer.d ./Core/Src/RingBuffer.o ./Core/Src/RingBuffer.su ./Core/Src/StateMachine.cyclo ./Core/Src/StateMachine.d ./Core/Src/StateMachine.o ./Core/Src/StateMachine.su ./Core/Src/freertos.cyclo ./Core/Src/freertos.d ./Core/Src/freertos.o ./Core/Src/freertos.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/sensorBMP388.cyclo ./Core/Src/sensorBMP388.d ./Core/Src/sensorBMP388.o ./Core/Src/sensorBMP388.su ./Core/Src/sensorLIS2MDL.cyclo ./Core/Src/sensorLIS2MDL.d ./Core/Src/sensorLIS2MDL.o ./Core/Src/sensorLIS2MDL.su ./Core/Src/sensorLSM6DSO32.cyclo ./Core/Src/sensorLSM6DSO32.d ./Core/Src/sensorLSM6DSO32.o ./Core/Src/sensorLSM6DSO32.su ./Core/Src/sensorShared.cyclo ./Core/Src/sensorShared.d ./Core/Src/sensorShared.o ./Core/Src/sensorShared.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_hal_timebase_tim.cyclo ./Core/Src/stm32f4xx_hal_timebase_tim.d ./Core/Src/stm32f4xx_hal_timebase_tim.o ./Core/Src/stm32f4xx_hal_timebase_tim.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/taskBMP388.cyclo ./Core/Src/taskBMP388.d ./Core/Src/taskBMP388.o ./Core/Src/taskBMP388.su ./Core/Src/taskLIS2MDL.cyclo ./Core/Src/taskLIS2MDL.d ./Core/Src/taskLIS2MDL.o ./Core/Src/taskLIS2MDL.su ./Core/Src/taskLSM6DSO32.cyclo ./Core/Src/taskLSM6DSO32.d ./Core/Src/taskLSM6DSO32.o ./Core/Src/taskLSM6DSO32.su ./Core/Src/taskShared.cyclo ./Core/Src/taskShared.d ./Core/Src/taskShared.o ./Core/Src/taskShared.su ./Core/Src/taskStateMachine.cyclo ./Core/Src/taskStateMachine.d ./Core/Src/taskStateMachine.o ./Core/Src/taskStateMachine.su ./Core/Src/taskUART.cyclo ./Core/Src/taskUART.d ./Core/Src/taskUART.o ./Core/Src/taskUART.su

.PHONY: clean-Core-2f-Src

