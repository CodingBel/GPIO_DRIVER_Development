/*
 * stm32407xx.h
 *
 *  Created on: Mar 17, 2024
 *      Author: belaa
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include<stdint.h>
#define __vo volatile

/*
 * Define the Base addresses for FLASH and SRAM memories
 * */
#define FLASH_BASE_ADDR 		0x08000000U
#define SRAM1_BASE_ADDR			0x20000000U
#define SRAM2_BASE_ADDR			((SRAM1_BASE_ADDR) + (112 * 1024))
#define SRAM


/*
 * Define the base addresses of	AHBx and APBx busses' memory addresses
 *
 * */
#define PERIPH_BASE       		0x40000000U

#define APB1PERIPH_BASE    		PERIPH_BASE
#define APB2PERIPH_BASE			0x40010000U
#define AHB1PERIPH_BASE			0x40020000U
#define AHB2PERIPH_BASE			0x50000000U

#define RCC_BASE_ADDR 			(AHB1PERIPH_BASE + 0x3800U)

#define ENABLE 			1
#define DISABLE 		0
#define SET				ENABLE
#define RESET			DISABLE

#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET

/*
 * Define the AHB1 bus peripherals
 * */
#define GPIOA_BASE_ADDR 		(AHB1PERIPH_BASE + 0x00U)
#define GPIOB_BASE_ADDR 		(AHB1PERIPH_BASE + 0x0400U)
#define GPIOC_BASE_ADDR 		(AHB1PERIPH_BASE + 0x0800U)
#define GPIOD_BASE_ADDR 		(AHB1PERIPH_BASE + 0x0C00U)
#define GPIOE_BASE_ADDR 		(AHB1PERIPH_BASE + 0x01000U)
#define GPIOF_BASE_ADDR 		(AHB1PERIPH_BASE + 0x01400U)
#define GPIOG_BASE_ADDR 		(AHB1PERIPH_BASE + 0x01800U)
#define GPIOH_BASE_ADDR 		(AHB1PERIPH_BASE + 0x01C00U)
#define GPIOI_BASE_ADDR 		(AHB1PERIPH_BASE + 0x02000U)


/*
 * Define the APB1 bus peripherals
 * */
#define I2C1_BASE_ADDR			(APB1PERIPH_BASE + 0x05400U)
#define I2C2_BASE_ADDR			(APB1PERIPH_BASE + 0x05800U)
#define I2C3_BASE_ADDR			(APB1PERIPH_BASE + 0x05C00U)

#define SPI2_BASE_ADDR			(APB1PERIPH_BASE + 0x03800U)
#define SPI3_BASE_ADDR			(APB1PERIPH_BASE + 0x03C00U)

#define UART2_BASE_ADDR			(APB1PERIPH_BASE + 0x04400U)
#define UART3_BASE_ADDR			(APB1PERIPH_BASE + 0x04800U)
#define UART4_BASE_ADDR			(APB1PERIPH_BASE + 0x04C00U)
#define UART5_BASE_ADDR			(APB1PERIPH_BASE + 0x05000U)


/*
 * Define the APB2 bus peripherals
 * */
#define EXTI_BASE_ADDR			(APB2PERIPH_BASE + 0x03C00U)
#define SPI1_BASE_ADDR          (APB2PERIPH_BASE + 0x03000U)
#define SYSCFG_BASE_ADDR		(APB2PERIPH_BASE + 0x03800U)

#define USART1_BASE_ADDR		(APB2PERIPH_BASE + 0x01000U)
#define USART6_BASE_ADDR		(APB2PERIPH_BASE + 0x01400U)

/*
 * GPIO Modes
 * */
#define GPIO_IN_MODE 		0x0
#define GPIO_OUT_MODE		0x1
#define GPIO_ALT_FN			0x2
#define GPIO_ANALOG			0x3


#define GPIO_OutType

/*
 * Define the GPIOA Registers
 * */
/// replaced by the struct as doing this for all GPIOs would be tedious and un necessary
/*
#define GPIOA_MODER				(GPIOA_BASE_ADDR + 0x0U)
#define GPIOA_OTYPER			(GPIOA_BASE_ADDR + 0x04U)
#define GPIOA_OSPEEDR 			(GPIOA_BASE_ADDR + 0x08U)
#define GPIOA_PUPDR				(GPIOA_BASE_ADDR + 0x0CU)
#define GPIOA_IDR				(GPIOA_BASE_ADDR + 0x010U)
#define GPIOA_ODR				(GPIOA_BASE_ADDR + 0x014U)
#define GPIOA_BSRR				(GPIOA_BASE_ADDR + 0x018U)
#define GPIOA_LCKR				(GPIOA_BASE_ADDR + 0x01CU)
#define GPIOA_AFRL				(GPIOA_BASE_ADDR + 0x020U)
#define GPIOA_AFRL				(GPIOA_BASE_ADDR + 0x024U)
*/

// struct for GPIO registers
typedef struct {
	__vo uint32_t MODER;     	/* GPIO Mode register Address offset: 0x00 */
	__vo uint32_t OTYPER;   	/* GPIO port output type register Address offset:  0x04 */
	__vo uint32_t OSPEEDR; 	 	/* GPIO output speed register. Address offset: *0x08 */
	__vo uint32_t PUPDR; 		/* GPIO port pull-up/pull-down register. Address offset: 0x0C*/
	__vo uint32_t IDR; 			/* GPIO port input data register. Address offset: 0x10 */
	__vo uint32_t ODR; 			/* GPIO port output data register. Address offset: 0x14*/
	__vo uint32_t BSRR; 		/* GPIO port input data register. Address offset: 0x18*/
	__vo uint32_t LCKR; 		/* GPIO port configuration lock register Address offset: 0x1C*/
	__vo uint32_t AFR[2]; 		/* GPIO GPIO alternate function Low & High register/ Address offset: 0x20 and 0x24 respectively*/
}GPIO_RegDef_t;
// GPIO_RegDef_t* pGPIOA = (GPIO_RegDef_t*)GPIOA_BASE_ADDR;

/*
 * GPIO Peripheral definitions for GPIO Registers. (i.e. Peripheral Base Address type casted to GPIO_RegDef_t)
 * */
#define GPIOA					((GPIO_RegDef_t*)GPIOA_BASE_ADDR)
#define GPIOB					((GPIO_RegDef_t*)GPIOB_BASE_ADDR)
#define GPIOC					((GPIO_RegDef_t*)GPIOC_BASE_ADDR)
#define GPIOD					((GPIO_RegDef_t*)GPIOD_BASE_ADDR)
#define GPIOE					((GPIO_RegDef_t*)GPIOE_BASE_ADDR)
#define GPIOF					((GPIO_RegDef_t*)GPIOF_BASE_ADDR)
#define GPIOG					((GPIO_RegDef_t*)GPIOG_BASE_ADDR)
#define GPIOH					((GPIO_RegDef_t*)GPIOH_BASE_ADDR)
#define GPIOI					((GPIO_RegDef_t*)GPIOI_BASE_ADDR)


/* struct for RCC registers*/
typedef struct{
	  __vo uint32_t CR;            /*!< TODO,     										Address offset: 0x00 */
	  __vo uint32_t PLLCFGR;       /*!< TODO,     										Address offset: 0x04 */
	  __vo uint32_t CFGR;          /*!< TODO,     										Address offset: 0x08 */
	  __vo uint32_t CIR;           /*!< TODO,     										Address offset: 0x0C */
	  __vo uint32_t AHB1RSTR;      /*!< TODO,     										Address offset: 0x10 */
	  __vo uint32_t AHB2RSTR;      /*!< TODO,     										Address offset: 0x14 */
	  __vo uint32_t AHB3RSTR;      /*!< TODO,     										Address offset: 0x18 */
	  uint32_t      RESERVED0;     /*!< Reserved, 0x1C                                                       */
	  __vo uint32_t APB1RSTR;      /*!< TODO,     										Address offset: 0x20 */
	  __vo uint32_t APB2RSTR;      /*!< TODO,     										Address offset: 0x24 */
	  uint32_t      RESERVED1[2];  /*!< Reserved, 0x28-0x2C                                                  */
	  __vo uint32_t AHB1ENR;       /*!< TODO,     										Address offset: 0x30 */
	  __vo uint32_t AHB2ENR;       /*!< TODO,     										Address offset: 0x34 */
	  __vo uint32_t AHB3ENR;       /*!< TODO,     										Address offset: 0x38 */
	  uint32_t      RESERVED2;     /*!< Reserved, 0x3C                                                       */
	  __vo uint32_t APB1ENR;       /*!< TODO,     										Address offset: 0x40 */
	  __vo uint32_t APB2ENR;       /*!< TODO,     										Address offset: 0x44 */
	  uint32_t      RESERVED3[2];  /*!< Reserved, 0x48-0x4C                                                  */
	  __vo uint32_t AHB1LPENR;     /*!< TODO,     										Address offset: 0x50 */
	  __vo uint32_t AHB2LPENR;     /*!< TODO,     										Address offset: 0x54 */
	  __vo uint32_t AHB3LPENR;     /*!< TODO,     										Address offset: 0x58 */
	  uint32_t      RESERVED4;     /*!< Reserved, 0x5C                                                       */
	  __vo uint32_t APB1LPENR;     /*!< TODO,     										Address offset: 0x60 */
	  __vo uint32_t APB2LPENR;     /*!< RTODO,     										Address offset: 0x64 */
	  uint32_t      RESERVED5[2];  /*!< Reserved, 0x68-0x6C                                                  */
	  __vo uint32_t BDCR;          /*!< TODO,     										Address offset: 0x70 */
	  __vo uint32_t CSR;           /*!< TODO,     										Address offset: 0x74 */
	  uint32_t      RESERVED6[2];  /*!< Reserved, 0x78-0x7C                                                  */
	  __vo uint32_t SSCGR;         /*!< TODO,     										Address offset: 0x80 */
	  __vo uint32_t PLLI2SCFGR;    /*!< TODO,     										Address offset: 0x84 */
	  __vo uint32_t PLLSAICFGR;    /*!< TODO,     										Address offset: 0x88 */
	  __vo uint32_t DCKCFGR;       /*!< TODO,     										Address offset: 0x8C */
	  __vo uint32_t CKGATENR;      /*!< TODO,     										Address offset: 0x90 */
	  __vo uint32_t DCKCFGR2;      /*!< TODO,     										Address offset: 0x94 */

}RCC_RegDef_t;
#define RCC 					((RCC_RegDef_t*)RCC_BASE_ADDR)

/* Create C macro FUNCTIONS for Enabling GPIO Peripheral Clocks */
/* Example: for enabling the GPIOA clock enable which is under AHB1ENR
 */
#define GPIOA_PCLK_EN() 	(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()   	(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN() 	(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()   	(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN() 	(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()   	(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN() 	(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()   	(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN() 	(RCC->AHB1ENR |= (1 << 8))

/*
 * Macros to RESET GPIO Peripherals
 * */
#define GPIOA_REG_RESET()  	do {(RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0));}while(0)
#define GPIOB_REG_RESET()  	do {(RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1));}while(0)
#define GPIOC_REG_RESET()  	do {(RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2));}while(0)
#define GPIOD_REG_RESET()  	do {(RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3));}while(0)
#define GPIOE_REG_RESET() 	do {(RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4));}while(0)
#define GPIOF_REG_RESET()  	do {(RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5));}while(0)
#define GPIOG_REG_RESET()  	do {(RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6));}while(0)
#define GPIOH_REG_RESET()  	do {(RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7));}while(0)
#define GPIOI_REG_RESET()  	do {(RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8));}while(0)



/*
 * C macro functions for Enabling I2Cx Clocks
 * */
#define I2C1_PCLK_EN() 		(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN() 		(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN() 		(RCC->APB1ENR |= (1 << 23))
/*
 * C macro functions for Enabling SPIx Clocks
 * */
#define SPI1_PCLK_EN()		(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()		(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()		(RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()		(RCC->APB2ENR |= (1 << 13))
#define SPI5_PCLK_EN()		(RCC->APB2ENR |= (1 << 20))
#define SPI6_PCLK_EN()		(RCC->APB2ENR |= (1 << 21))

/*
 * C macro functions for USARTx peripherals
 * */
#define USART1_PCLK_EN()	(RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()	(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()	(RCC->APB1ENR |= (1 << 18))
#define UART4_PCLCK_EN()	(RCC->APB1ENR |= (1 << 19))
#define UART5_PCLCK_EN()	(RCC->APB1ENR |= (1 << 20))
#define USART6_PCLK_EN()	(RCC->APB2ENR |= (1 << 5))

/*
 * C macro function for SYSCFG Clock
 * */
#define SYSCFG_PCLK_EN()	(RCC->APB2ENR |= (1 << 14))



/*
 * C macros for Clock disable
 * */

/*
 * Clock disable macros for GPIOx peripherals
 * */
#define GPIOA_PCLK_DI() 	(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()   	(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI() 	(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()   	(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI() 	(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()   	(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI() 	(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()   	(RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI() 	(RCC->AHB1ENR &= ~(1 << 8))


/*
 * Clock disable macros for I2Cx peripherals
 * */
#define I2C1_PCLK_DI() 		(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI() 		(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI() 		(RCC->APB1ENR &= ~(1 << 23))

/*
 * Clock disable macros for SPIx peripherals
 * */

#define SPI1_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 13))
#define SPI5_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 20))
#define SPI6_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 21))

/*
 * Clock disable macros for USARTx peripherals
 * */
#define USART1_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLCK_DI()	(RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLCK_DI()	(RCC->APB1ENR &= ~(1 << 20))
#define USART6_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 5))

/*
 * Clock disable macro for SYSCFG Clock
 * */
#define SYSCFG_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 14))

#endif /* INC_STM32F407XX_H_ */
