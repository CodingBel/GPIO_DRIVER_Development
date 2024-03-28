################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/Src/LEDtoggle.c \
../drivers/Src/stm32407xx_gpio_driver.c 

OBJS += \
./drivers/Src/LEDtoggle.o \
./drivers/Src/stm32407xx_gpio_driver.o 

C_DEPS += \
./drivers/Src/LEDtoggle.d \
./drivers/Src/stm32407xx_gpio_driver.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/Src/%.o drivers/Src/%.su drivers/Src/%.cyclo: ../drivers/Src/%.c drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I"C:/Users/belaa/Desktop/Driver Development/STmDriverDevelopment/stm32407xx_Drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-drivers-2f-Src

clean-drivers-2f-Src:
	-$(RM) ./drivers/Src/LEDtoggle.cyclo ./drivers/Src/LEDtoggle.d ./drivers/Src/LEDtoggle.o ./drivers/Src/LEDtoggle.su ./drivers/Src/stm32407xx_gpio_driver.cyclo ./drivers/Src/stm32407xx_gpio_driver.d ./drivers/Src/stm32407xx_gpio_driver.o ./drivers/Src/stm32407xx_gpio_driver.su

.PHONY: clean-drivers-2f-Src
