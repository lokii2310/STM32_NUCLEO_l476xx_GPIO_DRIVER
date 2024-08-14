/*
 * toggle_LED.c
 *
 *  Created on: Aug 12, 2024
 *      Author: hp
 */


#include "stm32nul476rg.h"

void delay(void)
{
	for(uint32_t i=0;i<50000;i++);
}




int main(void)
{
	GPIO_Handle_t GpioLed;
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConig.GPIO_PinPuPdContol = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioLed);

	while(1)
	{
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
		delay();
	}
	return 0;
}
