/*
 * stm32407xx_gpio_driver.c
 *
 *  Created on: Mar 27, 2024
 *      Author: Bela
 */

#include "stm32407xx_gpio_driver.h"

/***********************************************************************
 * @fn Name		- GPIO_PeriClockControl
 *
 * @brief		- This function Enables or Disables the GPIO Peripheral Clock
 *
 * @param[in]	- Base Address of the GPIO Peripheral
 * @param[in]	- ENABLE or DISABLE Macros
 *
 * @return 		- none
 *
 * @note		- none
 *
 */
void GPIO_PeriClockControl(GPIO_RegDef_t* pGPIOx, uint8_t EnOrDi){
	if (EnOrDi == ENABLE){
		if (pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		}

		else if (pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		}

		else if (pGPIOx == GPIOC){
			GPIOC_PCLK_EN();
		}

		else if (pGPIOx == GPIOD){
			GPIOD_PCLK_EN();
		}

		else if (pGPIOx == GPIOE){
			GPIOE_PCLK_EN();
		}

		else if (pGPIOx == GPIOF){
			GPIOF_PCLK_EN();
		}

		else if (pGPIOx == GPIOG){
			GPIOG_PCLK_EN();
		}

		else if (pGPIOx == GPIOH){
			GPIOH_PCLK_EN();
		}

		else if (pGPIOx == GPIOI){
			GPIOI_PCLK_EN();
		}
	}

	else {
		if (pGPIOx == GPIOA){
			GPIOA_PCLK_DI();
		}

		else if (pGPIOx == GPIOB){
			GPIOB_PCLK_DI();
		}

		else if (pGPIOx == GPIOC){
			GPIOC_PCLK_DI();
		}

		else if (pGPIOx == GPIOD){
			GPIOD_PCLK_DI();
		}

		else if (pGPIOx == GPIOE){
			GPIOE_PCLK_DI();
		}

		else if (pGPIOx == GPIOF){
			GPIOF_PCLK_DI();
		}

		else if (pGPIOx == GPIOG){
			GPIOG_PCLK_DI();
		}

		else if (pGPIOx == GPIOH){
			GPIOH_PCLK_DI();
		}

		else if (pGPIOx == GPIOI){
			GPIOI_PCLK_DI();
		}

	}
}


/***********************************************************************
 * @fn Name		- GPIO_Init
 *
 * @brief		- This function is used to Initialize the GPIO pin
 *
 * @param[in]	- Base Address of the GPIO_Handle struct
 *
 * @return 		- none
 *
 * @note		- none
 *
 */
void GPIO_Init(GPIO_Handle_t* pGPIO_Handle){
	// 1. Configure the Mode of the GPIO Pin

	uint32_t temp = 0;
	if (pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){
		temp = (pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber));  // 2x as the mode takes two bits
		pGPIO_Handle->pGPIOX->MODER &= ~(0x3 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIO_Handle->pGPIOX->MODER |= temp;

		temp = 0;
	}
	else {
			// For the interrupt modes
	}

	// 2. Configure the Speed
	temp = pGPIO_Handle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIO_Handle->pGPIOX->OSPEEDR &= ~(0x03 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIO_Handle->pGPIOX->OSPEEDR |= temp;

	temp = 0;

	// 3. Configure the PuPd settings
	temp = pGPIO_Handle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIO_Handle->pGPIOX->PUPDR &= ~(0x03 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIO_Handle->pGPIOX->PUPDR |= temp;

	temp = 0;

	// 4. Configure the Output Type
	temp = pGPIO_Handle->GPIO_PinConfig.GPIO_PinOPType << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber;
	pGPIO_Handle->pGPIOX->OTYPER &= ~(0x01 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIO_Handle->pGPIOX->OTYPER |= temp;

	temp = 0;

	// 5. Configure the Alternate Functionality
	if (pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){
		uint8_t index;
		uint8_t nibble;

		index  = pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber / 8;
		nibble = pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIO_Handle->pGPIOX->AFR[index] &= ~(0x0F << (4 * nibble));
		pGPIO_Handle->pGPIOX->AFR[index] |= (pGPIO_Handle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * nibble));
	}
}


/***********************************************************************
 * @fn Name		- GPIO_DeInit
 *
 * @brief		- This function is used to DeInitialize the GPIO pin
 *
 * @param[in]	- Base Address of the GPIO Peripheral
 *
 * @return 		- none
 *
 * @note		- none
 *
 */
void GPIO_DeInit(GPIO_RegDef_t* pGPIOx){
	if (pGPIOx == GPIOA){
		GPIOA_REG_RESET();
	}

	else if (pGPIOx == GPIOB){
		GPIOB_REG_RESET();
	}

	else if (pGPIOx == GPIOC){
		GPIOC_REG_RESET();
	}

	else if (pGPIOx == GPIOD){
		GPIOD_REG_RESET();
	}

	else if (pGPIOx == GPIOE){
		GPIOE_REG_RESET();
	}

	else if (pGPIOx == GPIOF){
		GPIOF_REG_RESET();
	}

	else if (pGPIOx == GPIOG){
		GPIOG_REG_RESET();
	}

	else if (pGPIOx == GPIOH){
		GPIOH_REG_RESET();
	}

	else if (pGPIOx == GPIOI){
		GPIOI_REG_RESET();
	}
}


/***********************************************************************
 * @fn Name		- GPIO_ReadFromInputPin
 *
 * @brief		- This function is used to read the value of a GPIO Pin
 *
 * @param[in]	- Base Address of the GPIO Peripheral
 * @param[in]	- The PinNumber of the GPIO Pin
 *
 * @return 		- 0 or 1
 *
 * @note		- none
 *
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
   uint8_t value;

   value = (uint8_t )((pGPIOx->IDR  >> PinNumber) & 0x1 ) ;

   return value;
}

/***********************************************************************
 * @fn Name		- GPIO_ReadFromInputPort
 *
 * @brief		- This function is used to read from a GPIO peripheral
 *
 * @param[in]	- Base Address of the GPIO Peripheral
 *
 * @return 		- Value of the GPIO IDR
 *
 * @note		- none
 *
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t* pGPIOx){
	uint16_t value = 0;
	value = (uint16_t)pGPIOx->IDR;

	return value;
}


/***********************************************************************
 * @fn Name		- GPIO_WriteToOutputPin
 *
 * @brief		- This function is used to write 1 or 0 to a GPIO Pin
 *
 * @param[in]	- Base Address of the GPIO Peripheral
 * @param[in]	- The GPIO Pin Number
 * @param[in]	- The value for the GPIO Pin
 *
 * @return 		- none
 *
 * @note		- none
 *
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber, uint8_t Value){
	if (Value == GPIO_PIN_SET){
		pGPIOx->ODR |= (1 << PinNumber);
	}

	else {
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}


/***********************************************************************
 * @fn Name		- GPIO_WriteToOutputPort
 *
 * @brief		- This function is used to write to a GPIO Port
 *
 * @param[in]	- Base Address of the GPIO Peripheral
 * @param[in]	- The value for the GPIO Port
 *
 * @return 		- none
 *
 * @note		- none
 *
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t* pGPIOx, uint16_t Value){
	pGPIOx->ODR = Value;
}


/***********************************************************************
 * @fn Name		- GPIO_ToggleOutputPin
 *
 * @brief		- This function is used to toggle the status of an output Pin
 *
 * @param[in]	- Base Address of the GPIO Peripheral
 * @param[in]	- The PinNumber of the GPIO Port
 *
 * @return 		- none
 *
 * @note		- none
 *
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber){

	pGPIOx->ODR ^= (1 << PinNumber);
}


/***********************************************************************
 * @fn Name		- GPIO_IRQConfig
 *
 * @brief		- This function is used to configure the Interrupt for a GPIO port
 *
 * @param[in]	- The IRQ number
 * @param[in]	- The Priority of interrupt
 * @param[in]	- ENABLE or DISABLE macros
 *
 * @return 		- none
 *
 * @note		- none
 *
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi){

}


/***********************************************************************
 * @fn Name		- GPIO_IRQHandling
 *
 * @brief		- This function is used to handle the GPIO interrupt
 *
 * @param[in]	- The GPIO Pin Number
 *
 *
 * @return 		- none
 *
 * @note		- none
 *
 */
void GPIO_IRQHandling(uint8_t PinNumber){

}
