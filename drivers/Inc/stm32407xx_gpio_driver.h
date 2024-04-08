/*
 * stm32407xx_gpio_driver.h
 *
 *  Created on: Mar 27, 2024
 *      Author: Bela
 */

#ifndef INC_STM32407XX_GPIO_DRIVER_H_
#define INC_STM32407XX_GPIO_DRIVER_H_


#include"stm32f407xx.h"


/*
 * @GPIO Pin Modes
 * GPIO Pin possible modes
 * */
#define GPIO_MODE_IN 		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANALOG	3

#define GPIO_MODE_IT_FT		4
#define GPIO_MODE_IT_RT		5
#define GPIO_MODE_IT_RFT	6


/*
 * @GPIO Pin OUT_TYPE
 * GPIO Pin Output types
 * */
#define GPIO_OP_TYPE_PP		0
#define GPIO_OP_TYPE_OD     1


/*
 * @GPIO Pin SPEED
 * GPIO Pin Output SPEED modes
 * */
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM 	1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIgh		3


/*
 * @GPIO Pin PUPD
 * GPIO Pin Pull UP and Pull DOWN Configurations
 * */
#define GPIO_NO_PUPD		0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2

/*
 * @GPIO Pin Number
 * GPIO Pin Numbers
 * */
#define GPIO_PIN_NO_0		0
#define GPIO_PIN_NO_1		1
#define GPIO_PIN_NO_2		2
#define GPIO_PIN_NO_3		3
#define GPIO_PIN_NO_4		4
#define GPIO_PIN_NO_5		5
#define GPIO_PIN_NO_6		6
#define GPIO_PIN_NO_7		7
#define GPIO_PIN_NO_8		8
#define GPIO_PIN_NO_9		9
#define GPIO_PIN_NO_10		10
#define GPIO_PIN_NO_11		11
#define GPIO_PIN_NO_12		12
#define GPIO_PIN_NO_13		13
#define GPIO_PIN_NO_14		14
#define GPIO_PIN_NO_15		15


/*
 * This is a configuration structure for a GPIO Pin
 * */
typedef struct {

	uint8_t GPIO_PinNumber;				/*!<Possible values from @GPIO Pin Number>*/
	uint8_t GPIO_PinMode;				/*!<Possible values from @GPIO Pin Modes>*/
	uint8_t GPIO_PinSpeed;				/*!<Possible values from @GPIO Pin SPEED>*/
	uint8_t GPIO_PinPuPdControl;		/*!<Possible values from @GPIO Pin PUPD>*/
	uint8_t GPIO_PinOPType;				/*!<Possible values from @GPIO Pin OUT_TYPE>*/
	uint8_t GPIO_PinAltFunMode;

}GPIO_PinConfig_t;


/*
 * This is a handle structure for the GPIO Peripheral
 * */
typedef struct{
	GPIO_RegDef_t *pGPIOX; 				/* This holds the base address of the GPIO port to which the pin belongs to*/
	GPIO_PinConfig_t GPIO_PinConfig; 	/* This holds the GPIO Pin Configuration settings */

}GPIO_Handle_t;


/*
 * APIs supported by this driver
 * */
/*
 * 	Peripheral Clock Setup
 * */
void GPIO_PeriClockControl(GPIO_RegDef_t* pGPIOx, uint8_t EnOrDi);

/*
 * GPIO Init and DeInit
 * */
void GPIO_Init(GPIO_Handle_t* pGPIO_Handle);
void GPIO_DeInit(GPIO_RegDef_t* pGPIOx);


/*
 * Read and Write to GPIO
 * */
uint8_t  GPIO_ReadFromInputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t* pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t* pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber);

/*
 * GPIO IRQ Configuration and ISR handling
 * */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig (uint8_t IRQNumber,uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);




#endif /* INC_STM32407XX_GPIO_DRIVER_H_ */
