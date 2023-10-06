/*
 * 003_led_button_ext.c
 *
 *  Created on: Sep 16, 2023
 *      Author: prati
 */


#include "stm32f446re.h"
#include "stm32f446re_gpio_driver.h"
#include <stdint.h>

#define LOW 			0
#define BTN_PRESSED 	LOW

void delay(void)
{
	for (uint32_t i = 0; i < 500000/2; ++i);
}

int main(void)
{
	GPIO_Handle_t GpioLed, GpioBtn;

	// Assign gpio peripheral to handle
	GpioLed.pGPIOx = GPIOA;
	GpioBtn.pGPIOx = GPIOB;

	// led config
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NO;

	// Enable GPIOD peripheral clock
	GPIO_PeriClockControl(GpioLed.pGPIOx, ENABLE);

	// Initialize GPIOA peripheral
	GPIO_Init(&GpioLed);

	// button config
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NO;

	// Enable GPIOC peripheral clock
	GPIO_PeriClockControl(GpioBtn.pGPIOx, ENABLE);

	// Initialize GPIOC peripheral
	GPIO_Init(&GpioBtn);

	// Led toggle // press blue button
	while(1)
	{
		if (GPIO_ReadFromInputPin(GpioBtn.pGPIOx, GPIO_PIN_NO_12) == BTN_PRESSED)
		{
			delay(); // to avoid button debouncing
			GPIO_ToggleOutputPin(GpioLed.pGPIOx, GPIO_PIN_NO_8);

		}
	}

	return 0;
}
