/*
 * stm32nul476rg_gpio_driver.c
 *
 *  Created on: Aug 10, 2024
 *      Author: hp
 */


#include "stm32nul476rg_gpio_driver.h"



/*
 *
 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx ==GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx ==GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx ==GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx ==GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx ==GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if(pGPIOx ==GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if(pGPIOx ==GPIOH)
		{
			GPIOH_PCLK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx ==GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx ==GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx ==GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if(pGPIOx ==GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else if(pGPIOx ==GPIOF)
		{
			GPIOF_PCLK_DI();
		}
		else if(pGPIOx ==GPIOG)
		{
			GPIOG_PCLK_DI();
		}
		else if(pGPIOx ==GPIOH)
		{
			GPIOH_PCLK_DI();
		}
	}

}







void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	//1. Configure the mode of the GPIO pin
	uint32_t temp=0;
	if (pGPIOHandle->GPIO_PinConig.GPIO_PinMode<= GPIO_MODE_ANALOG)
	{
		//the non interrupt mode of GPIO pGPIOx->MODER &= ~(0x3 << (2 * PinNumber));

		temp=(pGPIOHandle->GPIO_PinConig.GPIO_PinMode <<(2 *pGPIOHandle->GPIO_PinConig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &=~(0x3 << 2*pGPIOHandle->GPIO_PinConig.GPIO_PinNumber);		//clearing
		pGPIOHandle->pGPIOx->MODER |= temp;														//setting
		temp=0;
	}
	else{
		//later
	}

	temp=0;
	//2. Configure the speed
	temp= (pGPIOHandle->GPIO_PinConig.GPIO_PinSpeed <<(2 *pGPIOHandle->GPIO_PinConig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &=~(0x3 << 2*pGPIOHandle->GPIO_PinConig.GPIO_PinNumber);			//clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;														//setting

	temp=0;
	//3. Configure the pupd settings
	temp= (pGPIOHandle->GPIO_PinConig.GPIO_PinPuPdContol <<(2 *pGPIOHandle->GPIO_PinConig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &=~(0x3 << 2*pGPIOHandle->GPIO_PinConig.GPIO_PinNumber);			//clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp;															//setting

	temp=0;
	//4. Configure the optype
	temp= (pGPIOHandle->GPIO_PinConig.GPIO_PinOPType <<pGPIOHandle->GPIO_PinConig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &=~(0x1 << pGPIOHandle->GPIO_PinConig.GPIO_PinNumber);			//clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp;														//setting

	//5. Configure the alt function
	if (pGPIOHandle->GPIO_PinConig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		//CONFIGURE ONLY IF THE ALT FUNCTION REGISTERS.
		uint8_t temp1,temp2;

		temp1 = pGPIOHandle->GPIO_PinConig.GPIO_PinNumber/8;
		temp2 = pGPIOHandle->GPIO_PinConig.GPIO_PinNumber%8;
		pGPIOHandle->pGPIOx->AFR[temp1] |= (0xF<< (4* temp2));													//clearing
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConig.GPIO_PinAltFunMode << (4* temp2));		//setting

	}

}



void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if(pGPIOx ==GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if(pGPIOx ==GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if(pGPIOx ==GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if(pGPIOx ==GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if(pGPIOx ==GPIOF)
	{
		GPIOF_REG_RESET();
	}
	else if(pGPIOx ==GPIOG)
	{
		GPIOG_REG_RESET();
	}
	else if(pGPIOx ==GPIOH)
	{
		GPIOH_REG_RESET();
	}
}



uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber)& 0x00000001);		//LSB is extracted by shifting and masking then finally type cased
	return value;
}



uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;
}


void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		//write 1 to the output data register at the bit field corresponding to the pin value
		pGPIOx->ODR |= (1<<PinNumber);
	}
	else
	{
		pGPIOx->ODR &= ~(1<<PinNumber);
	}
}
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,  uint16_t Value)
{
	pGPIOx->ODR= Value;
}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR^=(1<<PinNumber);
}







void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);

