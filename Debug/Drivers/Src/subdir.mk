################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Src/STM32L0732RZ_SPI_driver.c \
../Drivers/Src/STM32L0732RZ_gpio_driver.c 

OBJS += \
./Drivers/Src/STM32L0732RZ_SPI_driver.o \
./Drivers/Src/STM32L0732RZ_gpio_driver.o 

C_DEPS += \
./Drivers/Src/STM32L0732RZ_SPI_driver.d \
./Drivers/Src/STM32L0732RZ_gpio_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Src/%.o Drivers/Src/%.su Drivers/Src/%.cyclo: ../Drivers/Src/%.c Drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DNUCLEO_L073RZ -DSTM32 -DSTM32L073RZTx -DSTM32L0 -c -I../Inc -I"C:/Users/ebedulskis/STM32CubeIDE/workspace_1.13.1/STM32L0732RZ_drivers/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-Src

clean-Drivers-2f-Src:
	-$(RM) ./Drivers/Src/STM32L0732RZ_SPI_driver.cyclo ./Drivers/Src/STM32L0732RZ_SPI_driver.d ./Drivers/Src/STM32L0732RZ_SPI_driver.o ./Drivers/Src/STM32L0732RZ_SPI_driver.su ./Drivers/Src/STM32L0732RZ_gpio_driver.cyclo ./Drivers/Src/STM32L0732RZ_gpio_driver.d ./Drivers/Src/STM32L0732RZ_gpio_driver.o ./Drivers/Src/STM32L0732RZ_gpio_driver.su

.PHONY: clean-Drivers-2f-Src

