/*
 * 001_led_toggle.c
 *
 *  Created on: Sep 16, 2023
 *      Author: prati
 */


#include "stm32f446re.h"
#include "stm32f446re_gpio_driver.h"

#include <stdint.h>

void delay(void)
{
	for (uint32_t i = 0; i < 500000; ++i);
}

int main(void)
{
	GPIO_Handle_t GpioLed;

	GpioLed.pGPIOx = GPIOA;

	// Reset
	GPIO_DeInit(GpioLed.pGPIOx);

	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
//	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
//	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NO;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NO;

	// Enable GPIOD peripheral clock
	GPIO_PeriClockControl(GpioLed.pGPIOx, ENABLE);

	// Initialize GPIOD peripheral
	GPIO_Init(&GpioLed);

	// Led toggle
	while(1)
	{
		GPIO_ToggleOutputPin(GpioLed.pGPIOx, GPIO_PIN_NO_5);
		delay();
	}

	// Reset
	GPIO_DeInit(GpioLed.pGPIOx);

	return 0;
}
