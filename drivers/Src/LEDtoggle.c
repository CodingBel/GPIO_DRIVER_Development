/*
 * LEDtoggle.c
 *
 *  Created on: Mar 28, 2024
 *      Author: Bela
 */

#include "stm32407xx_gpio_driver.h"
#include "stm32f407xx.h"

void delay (void){
	for (int i = 0; i < 300000 / 2; i++);
}

int main () {
	GPIO_Handle_t GpioLed;
	GpioLed.pGPIOX = GPIOD;

	GpioLed.GPIO_PinConfig.GPIO_PinMode  = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&GpioLed);

	while (1){
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
		delay ();
	}

	return 0;
}
