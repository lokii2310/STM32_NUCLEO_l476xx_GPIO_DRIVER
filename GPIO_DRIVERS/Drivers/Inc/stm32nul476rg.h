/*
 * stm32nul476rg.h
 *
 *  Created on: Aug 8, 2024
 *      Author: hp
 */

#ifndef INC_STM32NUL476RG_H_
#define INC_STM32NUL476RG_H_

#include<stdint.h>
#define __vo volatile

#define FLASH_BASEADDR    					0x08000000U					//Flash memory
#define SRAM1_BASEADDR						0x20000000U					//96kb
#define SRAM2_BASEADDR						0X20018000U					//32kb 0x20000000+96KB
#define ROM									0x1FFF0000U
#define SRAM 								SRAM1_BASEADDR


#define PERIPH_BASE							0x40000000U
#define APB1PERIPH_BASEADDR					PERIPH_BASE
#define APB2PERIPH_BASEADDR					0x40010000U
#define AHB1PERIPH_BASEADDR					0x40020000U
#define AHB2PERIPH_BASEADDR					0x48000000U


//Base address of the GPIOS
#define GPIOA_BASEADDR						(AHB2PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR						(AHB2PERIPH_BASEADDR + 0X0400)
#define GPIOC_BASEADDR						(AHB2PERIPH_BASEADDR + 0X0800)
#define GPIOD_BASEADDR						(AHB2PERIPH_BASEADDR + 0X0C00)
#define GPIOE_BASEADDR						(AHB2PERIPH_BASEADDR + 0X1000)
#define GPIOF_BASEADDR						(AHB2PERIPH_BASEADDR + 0X1400)
#define GPIOG_BASEADDR						(AHB2PERIPH_BASEADDR + 0X1800)
#define GPIOH_BASEADDR						(AHB2PERIPH_BASEADDR + 0X1C00)

#define RCC_BASEADDR 						(AHB1PERIPH_BASEADDR + 0x1000)	//0x4002 1000

//Base address of peripherals which are hanging on APB1 bus
#define I2C1_BASEADDR						(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR						(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR						(APB1PERIPH_BASEADDR + 0x5C00)

#define CAN1_BASEADDR						(APB1PERIPH_BASEADDR + 0x6400)

#define SPI2_BASEADDR						(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR						(APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR						(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR						(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR						(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR						(APB1PERIPH_BASEADDR + 0x5000)


//Base address of peripherals which are hanging on APB2 bus
#define EXTI_BASEADDR 						(APB2PERIPH_BASEADDR + 0X0400)
#define SYSCFG_BASEADDR						(APB2PERIPH_BASEADDR + 0x0000)
#define	SPI1_BASEADDR						(APB2PERIPH_BASEADDR + 0x3000)
#define USART1_BASEADDR						(APB2PERIPH_BASEADDR + 0x3800)


typedef struct
{
	__vo uint32_t MODER;					//__vo is volatalie keyword shortform
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];
	__vo uint32_t BRR;
	__vo uint32_t ASCR;				  /*!< GPIO Bit Reset register,               Address offset: 0x28      */

}GPIO_RegDef_t;

typedef struct
{
	__vo uint32_t CR;
	__vo uint32_t ICSCR;
	__vo uint32_t CFGR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t PLLSAI1CFGR;
	__vo uint32_t PLLSAI2CFGR;
	__vo uint32_t CIER;
	__vo uint32_t CIFR;
	__vo uint32_t CICR;
	uint32_t      RESERVED0;   // Reserved
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
	uint32_t      RESERVED1;   // Reserved
	__vo uint32_t APB1RSTR1;
	__vo uint32_t APB1RSTR2;
	__vo uint32_t APB2RSTR;
	uint32_t      RESERVED2;   // Reserved
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	uint32_t      RESERVED3;
	__vo uint32_t APB1ENR1;
	__vo uint32_t APB1ENR2;
	__vo uint32_t APB2ENR;
	uint32_t      RESERVED4;
	__vo uint32_t AHB1SMENR;
	__vo uint32_t AHB2SMENR;
	__vo uint32_t AHB3SMENR;
	uint32_t      RESERVED5;
	__vo uint32_t APB1SMENR1;
	__vo uint32_t APB1SMENR2;
	__vo uint32_t APB2SMENR;
	 uint32_t      RESERVED6;
	__vo uint32_t CCIPR;
	uint32_t      RESERVED7;
	__vo uint32_t BDCR;
	__vo uint32_t CSR;

}RCC_RegDef_t;



//peripheral definition (peripheral base type casted to xxx_RegDef_t)
#define GPIOA 		( (GPIO_RegDef_t*) GPIOA_BASEADDR )
#define GPIOB 		( (GPIO_RegDef_t*) GPIOB_BASEADDR )
#define GPIOC 		( (GPIO_RegDef_t*) GPIOC_BASEADDR )
#define GPIOD 		( (GPIO_RegDef_t*) GPIOD_BASEADDR )
#define GPIOE 		( (GPIO_RegDef_t*) GPIOE_BASEADDR )
#define GPIOF 		( (GPIO_RegDef_t*) GPIOF_BASEADDR )
#define GPIOG 		( (GPIO_RegDef_t*) GPIOG_BASEADDR )
#define GPIOH 		( (GPIO_RegDef_t*) GPIOH_BASEADDR )

#define RCC			((RCC_RegDef_t*)RCC_BASEADDR)

// 						ENABLE PCLK MACROS
//Clock enable macros for GPIOX peripherals
#define GPIOA_PCLK_EN()	( RCC->AHB2ENR |= (1<<0) )
#define GPIOB_PCLK_EN()	( RCC->AHB2ENR |= (1<<1) )
#define GPIOC_PCLK_EN()	( RCC->AHB2ENR |= (1<<2) )
#define GPIOD_PCLK_EN()	( RCC->AHB2ENR |= (1<<3) )
#define GPIOE_PCLK_EN()	( RCC->AHB2ENR |= (1<<4) )
#define GPIOF_PCLK_EN()	( RCC->AHB2ENR |= (1<<5) )
#define GPIOG_PCLK_EN()	( RCC->AHB2ENR |= (1<<6) )
#define GPIOH_PCLK_EN()	( RCC->AHB2ENR |= (1<<7) )


//Clock enable macros for I2Cx peripherals
#define I2C1_PCLK_EN()	( RCC->APB1ENR |= (1<<21) )
#define I2C2_PCLK_EN()	( RCC->APB1ENR |= (1<<22) )
#define I2C3_PCLK_EN()	( RCC->APB1ENR |= (1<<23) )


//Clock enable macros for I2Cx peripherals
#define SPI1_PCLK_EN() 	( RCC->APB2ENR |= (1<<12) )
#define SPI2_PCLK_EN() 	( RCC->APB1ENR |= (1<<14) )
#define SPI3_PCLK_EN() 	( RCC->APB1ENR |= (1<<15) )


//Clock enable macros for I2Cx peripherals
#define USART1_PCLK_EN() ( RCC-> APB2ENR |= (1<<14) )
#define USART2_PCLK_EN() ( RCC-> APB1ENR |= (1<<17) )
#define USART3_PCLK_EN() ( RCC-> APB1ENR |= (1<<18) )
#define UART4_PCLK_EN() ( RCC-> APB1ENR |= (1<<19) )
#define UART5_PCLK_EN() ( RCC-> APB1ENR |= (1<<20) )


//Clock Enable macros for SYSCFG peripherals
#define SYSCFG_PCLK_EN() 	( RCC->APB2ENR |= (1<<0) )


//						CLK DISABLE MACROS

//Clock Disable macros for GPIOX peripherals
#define GPIOA_PCLK_DI()	( RCC->AHB2ENR &= ~(1<<0) )
#define GPIOB_PCLK_DI()	( RCC->AHB2ENR &= ~(1<<1) )
#define GPIOC_PCLK_DI()	( RCC->AHB2ENR &= ~(1<<2) )
#define GPIOD_PCLK_DI()	( RCC->AHB2ENR &= ~(1<<3) )
#define GPIOE_PCLK_DI()	( RCC->AHB2ENR &= ~(1<<4) )
#define GPIOF_PCLK_DI()	( RCC->AHB2ENR &= ~(1<<5) )
#define GPIOG_PCLK_DI()	( RCC->AHB2ENR &= ~(1<<6) )
#define GPIOH_PCLK_DI()	( RCC->AHB2ENR &= ~(1<<7) )

//Clock Disable macros for I2Cx peripherals
#define I2C1_PCLK_DI()	( RCC->APB1ENR &= ~(1<<21) )
#define I2C2_PCLK_DI()	( RCC->APB1ENR &= ~(1<<22) )
#define I2C3_PCLK_DI()	( RCC->APB1ENR &= ~(1<<23) )


//Clock Disable macros for I2Cx peripherals
#define SPI1_PCLK_DI() 	( RCC->APB2ENR &= ~(1<<12) )
#define SPI2_PCLK_DI() 	( RCC->APB1ENR &= ~(1<<14) )
#define SPI3_PCLK_DI() 	( RCC->APB1ENR &= ~(1<<15) )


//Clock Disable macros for I2Cx peripherals
#define USART1_PCLK_DI() ( RCC-> APB2ENR &= ~(1<<14) )
#define USART2_PCLK_DI() ( RCC-> APB1ENR &= ~(1<<17) )
#define USART3_PCLK_DI() ( RCC-> APB1ENR &= ~(1<<18) )
#define UART4_PCLK_DI() ( RCC-> APB1ENR &= ~(1<<19) )
#define UART5_PCLK_DI() ( RCC-> APB1ENR &= ~(1<<20) )


//Clock Enable macros for SYSCFG peripherals
#define SYSCFG_PCLK_DI() 	( RCC->APB2ENR &= ~(1<<0) )

//Macros to reset GPIOx peripherals
#define GPIOA_REG_RESET()			do{ (RCC->AHB2RSTR |=(1<<0));  (RCC->AHB2RSTR &=~(1<<0)); } while(0)
#define GPIOB_REG_RESET()			do{ (RCC->AHB2RSTR |=(1<<1));  (RCC->AHB2RSTR &=~(1<<1)); } while(0)
#define GPIOC_REG_RESET()			do{ (RCC->AHB2RSTR |=(1<<2));  (RCC->AHB2RSTR &=~(1<<2)); } while(0)
#define GPIOD_REG_RESET()			do{ (RCC->AHB2RSTR |=(1<<3));  (RCC->AHB2RSTR &=~(1<<3)); } while(0)
#define GPIOE_REG_RESET()			do{ (RCC->AHB2RSTR |=(1<<4));  (RCC->AHB2RSTR &=~(1<<4)); } while(0)
#define GPIOF_REG_RESET()			do{ (RCC->AHB2RSTR |=(1<<5));  (RCC->AHB2RSTR &=~(1<<5)); } while(0)
#define GPIOG_REG_RESET()			do{ (RCC->AHB2RSTR |=(1<<6));  (RCC->AHB2RSTR &=~(1<<6)); } while(0)
#define GPIOH_REG_RESET()			do{ (RCC->AHB2RSTR |=(1<<7));  (RCC->AHB2RSTR &=~(1<<7)); } while(0)


//some generic macros
#define ENABLE 			1
#define DISABLE 		0
#define SET 			ENABLE
#define RESET 			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET



#include "stm32nul476rg_gpio_driver.h"

#endif /* INC_STM32NUL476RG_H_ */
