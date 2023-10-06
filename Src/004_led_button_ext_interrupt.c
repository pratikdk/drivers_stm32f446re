/*
 * 004_led_button_ext_interrupt.c
 *
 *  Created on: Sep 17, 2023
 *      Author: prati
 */


#include "stm32f446re.h"
#include "stm32f446re_gpio_driver.h"
#include <string.h>
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
	memset(&GpioLed, 0, sizeof(GpioLed));
	memset(&GpioBtn, 0, sizeof(GpioBtn));

	// Assign gpio peripheral to handle
	GpioLed.pGPIOx = GPIOA;
	GpioBtn.pGPIOx = GPIOB;

	// led config
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NO;

	// Enable GPIOA peripheral clock
	GPIO_PeriClockControl(GpioLed.pGPIOx, ENABLE);

	// Initialize GPIOA peripheral
	GPIO_Init(&GpioLed);

	// button config
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NO;

	// Enable GPIOB peripheral clock
	GPIO_PeriClockControl(GpioBtn.pGPIOx, ENABLE);

	// Initialize GPIOB peripheral
	GPIO_Init(&GpioBtn);

	// Set interrupt priority for IRQ EXTI15_10(12) corresponding to pin 12
	GPIO_IRQPriorityConfig(EXTI15_10, NVIC_IRQ_PRI_15);
	// Set interrupt enable for IRQ EXTI15_10(12) corresponding to pin 12
	GPIO_IRQInterruptConfig(EXTI15_10, ENABLE);

	while(1);

	return 0;
}

void EXTI15_10_IRQHandler(void)
{
	delay();
	GPIO_IRQHandling(GPIO_PIN_NO_12);
	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_8);
}
