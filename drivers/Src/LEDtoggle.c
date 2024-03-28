/*
 * LEDtoggle.c
 *
 *  Created on: Mar 28, 2024
 *      Author: Bela
 */

#include "stm32407xx_gpio_driver.h"
#include "stm32f407xx.h"

void delay (void){
	for (int i = 0; i < 250000; i++);
}

int main () {

	GPIO_Handle_t GpioLed;
	GpioLed.pGPIOX = GPIOD;

	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode  = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;


	GPIO_Handle_t UserBtn;
	UserBtn.pGPIOX = GPIOA;

	UserBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	UserBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	UserBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;



	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&GpioLed);

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&UserBtn);

	while (1){
		if (GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0)){
			//delay();
			GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
		}


	}

	return 0;
}
