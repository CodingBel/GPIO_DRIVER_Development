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
		if (pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT){
			// This is a Falling edge trigger
			// 1. Configure the FTSR (Falling Trigger Selection Register)

			EXTI->FTSR |=  (1 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR &= ~(1 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
		}

		else if (pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){
			// This is a rising edge trigger
			// 1. Configure the RTSR (Rising Trigger Selection Register)

			EXTI->RTSR |=  (1 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR &= ~(1 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
		}

		else if (pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT){
			// This is a rising and falling edge trigger
			EXTI->FTSR |=  (1 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |=  (1 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
		}

		// 2. Configure the GPIO Port Selection in SYSCFG_EXTICR
		uint8_t index  = pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t nibble = pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber % 4;

		uint8_t portCode = GPIO_BASEADDR_TO_CODE (pGPIO_Handle->pGPIOX);

		SYSCFG_PCLK_EN(); 	// Enable the SYSCFG clock
		SYSCFG->EXTICR[index] = portCode << ((nibble * 4)); // X4 for selecting the nibble

		// 3. Enable the EXTI interrupt delivery using IMR (Interrupt Mask Register)
		EXTI->IMR |= (1 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
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
 * @fn Name		- GPIO_IRQ_Interrupt_Config
 *
 * @brief		- This function is used to Enable and disable the Interrupt for a GPIO Peripherals
 *
 * @param[in]	- The IRQ (Interrupt Request)number
 * @param[in]	- ENABLE or DISABLE macros
 *
 * @return 		- none
 *
 * @note		- none
 *
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){
// This is for the MCU's IRQ number and ARM's NVIC mapping
// ARM Side configuration
// However, the MCU only utilizes 81 IRQ for it's peripherals' interrupts in its Vector table.

	if (EnorDi == ENABLE){ // For enabling the interrupt registers
		if (IRQNumber <= 31){
			// IRQNumber lies in the ISER0 Register
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if (IRQNumber > 31 && IRQNumber < 64){
			// IRQNumber lies in the ISER1 Register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if (IRQNumber >= 64 && IRQNumber < 96){
			// IRQNumber lies in the ISER2 Register
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}

	else { // For Disabling the interrupts by using the ICER (Clear Interrupt) register
		if (IRQNumber <= 31){ // for IRQs 0 - 31
			// IRQNumber lies in the ICER0 Register
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if (IRQNumber > 31 && IRQNumber < 64){ // for IRQs 32 - 63
			// IRQNumber lies in the ICER1 Register
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		else if (IRQNumber >= 64 && IRQNumber < 96){ // for IRQs 64 - 88
			// IRQNumber lies in the ICER2 Register
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
	}
}


/***********************************************************************
 * NOTE: The ARM Cortex Mx processor has 60 32-bit interrupt registers from IPR0 - IPR059
 * Each register holds the priority value for 4 IRQs and Each byte in the IPRx register holds
 * the priority value for that IRQ ranges from 0 - 15 which is MCU Specific
 * 0 - 15 because, In each byte only the 4 MSB bits are used. The 4 LSB bits are N/A Non applicable
 *
 * @fn Name		- GPIO_IRQ_Priotity_Config
 *
 * @brief		- This function is used to set the priority for GPIO interrupts
 *
 * @param[in]	- The IRQ (Interrupt Request)number
 * @param[in]	- The IRQ priority value
 *
 * @return 		- none
 *
 * @note		- none
 *
 */
void GPIO_IRQPriorityConfig (uint8_t IRQNumber, uint8_t IRQPriority){
	uint8_t IPR_reg = IRQNumber / 4;  			// For selecting 0 - 59 IPRx registers
	uint8_t byte_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * byte_section) + (8 - NO_OF_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASE_ADDR + IPR_reg) |= (IRQPriority << shift_amount);
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

	if ((EXTI->PR) & (1 << PinNumber)){
		// Clear the EXTI PR register corresponding to the pin
		// Clear by writing 1
		EXTI->PR |= (1 << PinNumber);
	}
}
