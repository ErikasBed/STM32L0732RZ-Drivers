################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/009_SPI_MESSAGE_RCV_IT.c \
../Src/syscalls.c \
../Src/sysmem.c 

OBJS += \
./Src/009_SPI_MESSAGE_RCV_IT.o \
./Src/syscalls.o \
./Src/sysmem.o 

C_DEPS += \
./Src/009_SPI_MESSAGE_RCV_IT.d \
./Src/syscalls.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DNUCLEO_L073RZ -DSTM32 -DSTM32L073RZTx -DSTM32L0 -c -I../Inc -I"C:/Users/ebedulskis/STM32CubeIDE/workspace_1.13.1/STM32L0732RZ_drivers/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/009_SPI_MESSAGE_RCV_IT.cyclo ./Src/009_SPI_MESSAGE_RCV_IT.d ./Src/009_SPI_MESSAGE_RCV_IT.o ./Src/009_SPI_MESSAGE_RCV_IT.su ./Src/syscalls.cyclo ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.cyclo ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su

.PHONY: clean-Src

