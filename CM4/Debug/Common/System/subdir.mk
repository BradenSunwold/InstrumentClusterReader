################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/home/braden/STM32CubeIDE/workspace_1.4.0/InstrumentClusterReader/Common/System/system_stm32mp1xx.c 

C_DEPS += \
./Common/System/system_stm32mp1xx.d 

OBJS += \
./Common/System/system_stm32mp1xx.o 


# Each subdirectory must supply rules for building sources it contributes
Common/System/system_stm32mp1xx.o: /home/braden/STM32CubeIDE/workspace_1.4.0/InstrumentClusterReader/Common/System/system_stm32mp1xx.c Common/System/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DMETAL_MAX_DEVICE_REGIONS=2 -DUSE_HAL_DRIVER -DSTM32MP157Cxx -DCORE_CM4 -DDEBUG -DNO_ATOMIC_64_SUPPORT -DMETAL_INTERNAL -DVIRTIO_SLAVE_ONLY -c -I../OPENAMP -I../Core/Inc -I../../Middlewares/Third_Party/OpenAMP/open-amp/lib/include -I../../Middlewares/Third_Party/OpenAMP/libmetal/lib/include -I../../Drivers/STM32MP1xx_HAL_Driver/Inc -I../../Drivers/STM32MP1xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32MP1xx/Include -I../../Middlewares/Third_Party/OpenAMP/virtual_driver -I../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

