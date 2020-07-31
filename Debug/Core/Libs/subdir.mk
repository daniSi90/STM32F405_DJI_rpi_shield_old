################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Libs/Quaternion.c \
../Core/Libs/TR_One_HAL.c \
../Core/Libs/bno080.c 

C_DEPS += \
./Core/Libs/Quaternion.d \
./Core/Libs/TR_One_HAL.d \
./Core/Libs/bno080.d 

OBJS += \
./Core/Libs/Quaternion.o \
./Core/Libs/TR_One_HAL.o \
./Core/Libs/bno080.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Libs/Quaternion.o: ../Core/Libs/Quaternion.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32F405xx -DUSE_HAL_DRIVER -DUSE_FULL_LL_DRIVER -DDEBUG -c -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Core/Libs -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Libs/Quaternion.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Libs/TR_One_HAL.o: ../Core/Libs/TR_One_HAL.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32F405xx -DUSE_HAL_DRIVER -DUSE_FULL_LL_DRIVER -DDEBUG -c -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Core/Libs -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Libs/TR_One_HAL.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Libs/bno080.o: ../Core/Libs/bno080.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32F405xx -DUSE_HAL_DRIVER -DUSE_FULL_LL_DRIVER -DDEBUG -c -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Core/Libs -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Libs/bno080.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

