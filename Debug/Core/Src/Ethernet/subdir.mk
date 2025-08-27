################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Ethernet/W5500_SPI.c \
../Core/Src/Ethernet/dhcp.c \
../Core/Src/Ethernet/socket.c \
../Core/Src/Ethernet/wizchip_conf.c 

OBJS += \
./Core/Src/Ethernet/W5500_SPI.o \
./Core/Src/Ethernet/dhcp.o \
./Core/Src/Ethernet/socket.o \
./Core/Src/Ethernet/wizchip_conf.o 

C_DEPS += \
./Core/Src/Ethernet/W5500_SPI.d \
./Core/Src/Ethernet/dhcp.d \
./Core/Src/Ethernet/socket.d \
./Core/Src/Ethernet/wizchip_conf.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Ethernet/%.o Core/Src/Ethernet/%.su Core/Src/Ethernet/%.cyclo: ../Core/Src/Ethernet/%.c Core/Src/Ethernet/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Core/Src/Ethernet -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Ethernet

clean-Core-2f-Src-2f-Ethernet:
	-$(RM) ./Core/Src/Ethernet/W5500_SPI.cyclo ./Core/Src/Ethernet/W5500_SPI.d ./Core/Src/Ethernet/W5500_SPI.o ./Core/Src/Ethernet/W5500_SPI.su ./Core/Src/Ethernet/dhcp.cyclo ./Core/Src/Ethernet/dhcp.d ./Core/Src/Ethernet/dhcp.o ./Core/Src/Ethernet/dhcp.su ./Core/Src/Ethernet/socket.cyclo ./Core/Src/Ethernet/socket.d ./Core/Src/Ethernet/socket.o ./Core/Src/Ethernet/socket.su ./Core/Src/Ethernet/wizchip_conf.cyclo ./Core/Src/Ethernet/wizchip_conf.d ./Core/Src/Ethernet/wizchip_conf.o ./Core/Src/Ethernet/wizchip_conf.su

.PHONY: clean-Core-2f-Src-2f-Ethernet

