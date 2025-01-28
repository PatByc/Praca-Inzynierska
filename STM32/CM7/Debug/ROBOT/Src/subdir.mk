################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ROBOT/Src/Robotics_Calculations.c 

OBJS += \
./ROBOT/Src/Robotics_Calculations.o 

C_DEPS += \
./ROBOT/Src/Robotics_Calculations.d 


# Each subdirectory must supply rules for building sources it contributes
ROBOT/Src/%.o ROBOT/Src/%.su ROBOT/Src/%.cyclo: ../ROBOT/Src/%.c ROBOT/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DCORE_CM7 -DUSE_HAL_DRIVER -DSTM32H755xx -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -I"C:/Users/dariu/OneDrive/Dokumenty/Inzynierka/InzynierkaV2-projekt/CM7/ROBOT/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-ROBOT-2f-Src

clean-ROBOT-2f-Src:
	-$(RM) ./ROBOT/Src/Robotics_Calculations.cyclo ./ROBOT/Src/Robotics_Calculations.d ./ROBOT/Src/Robotics_Calculations.o ./ROBOT/Src/Robotics_Calculations.su

.PHONY: clean-ROBOT-2f-Src

