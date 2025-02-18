/*
 * stm32nul476rg_gpio_driver.h
 *
 *  Created on: Aug 10, 2024
 *      Author: hp
 */

#ifndef INC_STM32NUL476RG_GPIO_DRIVER_H_
#define INC_STM32NUL476RG_GPIO_DRIVER_H_

#include "stm32nul476rg.h"


typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;    			/*!< possible values from @GPIO_PIN_MODE >*/
	uint8_t GPIO_PinSpeed;				/*!< possible values from @GPIO_PIN_SPEED >*/
	uint8_t GPIO_PinPuPdContol;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

typedef struct
{
	GPIO_RegDef_t *pGPIOx; 		//this holds the base address of the gpio port to which port to which pin belong
	GPIO_PinConfig_t GPIO_PinConig;

}GPIO_Handle_t;


// GPIO pin possible NUMBERS
// @GPIO_PIN_NUMBERS
#define GPIO_PIN_NO_0 		0
#define GPIO_PIN_NO_1 		1
#define GPIO_PIN_NO_2 		2
#define GPIO_PIN_NO_3 		3
#define GPIO_PIN_NO_4 		4
#define GPIO_PIN_NO_5 		5
#define GPIO_PIN_NO_6 		6
#define GPIO_PIN_NO_7 		7
#define GPIO_PIN_NO_8 		8
#define GPIO_PIN_NO_9 		9
#define GPIO_PIN_NO_10 		10
#define GPIO_PIN_NO_11 		11
#define GPIO_PIN_NO_12		12
#define GPIO_PIN_NO_13		13
#define GPIO_PIN_NO_14 		14
#define GPIO_PIN_NO_15 		15


// GPIO pin possible modes
// @GPIO_PIN_MODE
#define GPIO_MODE_IN 		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANALOG 	3
#define GPIO_MODE_IT_FT		4
#define GPIO_MODE_IT_RT		5
#define GPIO_MODE_IT_RFT	6

// GPIO possible output types
#define GPIO_OP_TYPE_PP		0
#define GPIO_OP_TYPE_OD		1

//GPIO pin possible output speeds
//@GPIO_PIN_SPEED
#define GPIO_SPEED_LOW 		0
#define GPIO_SPEED_MEDIUM  	1
#define GPIO_SPEED_FAST		3
#define GPIO_SPEED_HIGH		4

//GPIO pin PULL UP and PULL DOWN configuration macros
#define GPIO_NO_PUPD		0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2






void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);







void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);






uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,  uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);











void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);













#endif /* INC_STM32NUL476RG_GPIO_DRIVER_H_ */
