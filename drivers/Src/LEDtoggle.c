/*
 * LEDtoggle.c
 *
 *  Created on: Mar 28, 2024
 *      Author: Bela
 */

#include "stm32407xx_gpio_driver.h"
#include "stm32f407xx.h"

#define HIGH 1
#define LOW 0
#define BTN_PRESSED LOW

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}


int main(void)
{

	GPIO_Handle_t extLed, extBtn;

	// External Led at PA8
	extLed.pGPIOX = GPIOA;
	extLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	extLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	extLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	extLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	extLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA,ENABLE);

	GPIO_Init(&extLed);


	// External Button at PA14 Active low
	extBtn.pGPIOX = GPIOB;
	extBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	extBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	extBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	extBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOB,ENABLE);
	GPIO_Init(&extBtn);


	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOB,GPIO_PIN_NO_14) == LOW)
		{
			delay();
			GPIO_ToggleOutputPin(GPIOA,GPIO_PIN_NO_8);

		}
	}
	return 0;
}
