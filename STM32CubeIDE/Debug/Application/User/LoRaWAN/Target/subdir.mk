################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/Users/chaudhrymustafajawad/STM32CubeIDE/Latest/LoRaWAN_End_Node_FreeRTOS\ new/LoRaWAN/Target/radio_board_if.c 

OBJS += \
./Application/User/LoRaWAN/Target/radio_board_if.o 

C_DEPS += \
./Application/User/LoRaWAN/Target/radio_board_if.d 


# Each subdirectory must supply rules for building sources it contributes
Application/User/LoRaWAN/Target/radio_board_if.o: /Users/chaudhrymustafajawad/STM32CubeIDE/Latest/LoRaWAN_End_Node_FreeRTOS\ new/LoRaWAN/Target/radio_board_if.c Application/User/LoRaWAN/Target/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32WL55xx -c -I../../Core/Inc -I"/Users/chaudhrymustafajawad/STM32CubeIDE/Latest/LoRaWAN_End_Node_FreeRTOS new/STM32CubeIDE/Application/User/Core/Inc" -I"/Users/chaudhrymustafajawad/STM32CubeIDE/Latest/LoRaWAN_End_Node_FreeRTOS new/STM32CubeIDE/Drivers/CMSIS/DSP/Include" -I"/Users/chaudhrymustafajawad/STM32CubeIDE/Latest/LoRaWAN_End_Node_FreeRTOS new/STM32CubeIDE/Drivers/STM32WLxx_HAL_Driver/Inc" -I"/Users/chaudhrymustafajawad/STM32CubeIDE/Latest/LoRaWAN_End_Node_FreeRTOS new/STM32CubeIDE/Drivers/STM32WLxx_HAL_Driver" -I../../LoRaWAN/App -I../../LoRaWAN/Target -I../../Drivers/STM32WLxx_HAL_Driver/Inc/Legacy -I../../Utilities/trace/adv_trace -I../../Utilities/misc -I../../Utilities/timer -I../../Utilities/lpm/tiny_lpm -I../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../../Middlewares/Third_Party/LoRaWAN/LmHandler/Packages -I../../Drivers/CMSIS/Device/ST/STM32WLxx/Include -I../../Middlewares/Third_Party/LoRaWAN/Crypto -I../../Middlewares/Third_Party/LoRaWAN/Mac/Region -I../../Middlewares/Third_Party/LoRaWAN/Mac -I../../Middlewares/Third_Party/LoRaWAN/LmHandler -I../../Middlewares/Third_Party/LoRaWAN/Utilities -I../../Middlewares/Third_Party/SubGHz_Phy -I../../Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver -I../../Drivers/CMSIS/Include -I../../Drivers/BSP/STM32WLxx_Nucleo -I/Users/chaudhrymustafajawad/STM32CubeIDE/Latest/LoRaWAN_End_Node_FreeRTOS/Utilities/sequencer -I../../Drivers/STM32WLxx_HAL_Driver/Inc -I../../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/PrivateInclude/ -I../../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/ -I../../Middlewares/Third_Party/ARM_CMSIS/CMSIS/Core/Include/ -I../../Middlewares/Third_Party/ARM_CMSIS/CMSIS/Core_A/Include/ -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Application/User/LoRaWAN/Target/radio_board_if.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Application-2f-User-2f-LoRaWAN-2f-Target

clean-Application-2f-User-2f-LoRaWAN-2f-Target:
	-$(RM) ./Application/User/LoRaWAN/Target/radio_board_if.cyclo ./Application/User/LoRaWAN/Target/radio_board_if.d ./Application/User/LoRaWAN/Target/radio_board_if.o ./Application/User/LoRaWAN/Target/radio_board_if.su

.PHONY: clean-Application-2f-User-2f-LoRaWAN-2f-Target

