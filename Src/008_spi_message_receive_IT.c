/*
 * 009_spi_message_receive_IT.c
 *
 *  Created on: Sep 27, 2023
 *      Author: prati
 */

/*
 * PB14 --> SPI2_MISO
 * PB15 --> SPI2_MOSI
 * PB13 -> SPI2_SCLK
 * PB12 --> SPI2_NSS
 * ALT function mode : 5
 */

#include "stm32f446re_gpio_driver.h"
#include "stm32f446re_spi_driver.h"
#include "stm32f446re.h"
#include <string.h>
#include <stdio.h>

#define MAX_LEN			500U

SPI_Handle_t SPI2Handle;

char RcvBuff[MAX_LEN];
volatile char readByte;

volatile uint8_t rcvStop = 0;
volatile uint8_t dataAvailable = 0;

extern void initialise_monitor_handles(void);

void delay(void)
{
	for (uint32_t i = 0; i < 500000/2; ++i);
}

void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPI_GPIOPins;

	SPI_GPIOPins.pGPIOx = GPIOB;
	SPI_GPIOPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPI_GPIOPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPI_GPIOPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPI_GPIOPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NO;
	SPI_GPIOPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// SCLK
	SPI_GPIOPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPI_GPIOPins);

	// MOSI
	SPI_GPIOPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPI_GPIOPins);

	// MISO
	SPI_GPIOPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPI_GPIOPins);

	// NSS
	SPI_GPIOPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPI_GPIOPins);
}

void SPI2_Inits(void)
{
	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_DI;

	SPI_Init(&SPI2Handle);
}

void GPIO_InterruptPinInit(void) // Configures GPIOD, pin 6 to IT_FT
{
	GPIO_Handle_t ITPin;
	memset(&ITPin, 0, sizeof(ITPin));

	ITPin.pGPIOx = GPIOA;
	ITPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_11;
	ITPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	ITPin.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	ITPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NO;
	ITPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;

	GPIO_Init(&ITPin);

	GPIO_IRQPriorityConfig(EXTI15_10, NVIC_IRQ_PRI_15);
	GPIO_IRQInterruptConfig(EXTI15_10, ENABLE);
}

int main(void)
{
	initialise_monitor_handles();

	uint8_t dummy = 0;

	GPIO_InterruptPinInit(); // high to low transition by slave, raises an interrupt on pd6

	SPI2_GPIOInits();

	SPI2_Inits(); // initialize()

	/*
	* making SSOE 1 does NSS output enable.
	* The NSS pin is automatically managed by the hardware.
	* i.e when SPE=1 , NSS will be pulled to low
	* and NSS pin will be high when SPE=0
	*/
	SPI_SSOEConfig(SPI2, ENABLE);

	while(1)
	{
		rcvStop = 0;
		while (!dataAvailable); //wait till data available interrupt from transmitter device(slave) on pd6
		GPIO_IRQInterruptConfig(EXTI15_10,DISABLE); // disable interrupts on pd6 until tx complete

		// enable IRQ to receive spi2 interrupts on NVIC
		SPI_IRQInterruptConfig(SPI2_IRQ,ENABLE);
		SPI_PeriControl(SPI2, ENABLE); // SPI2 enable

		while(!rcvStop) // keep reading byte from slave until entire string is sent, IT processed
		{
			while(SPI_SendDataIT(&SPI2Handle, &dummy, 1) == SPI_BUSY_IN_TX);
			while(SPI_ReceiveDataIT(&SPI2Handle, &readByte, 1) == SPI_BUSY_IN_RX);
		}

		// Disable SPI2 peripheral
		while (SPI_GetFlagStatus(SPI2, SPI_SR_BSY_FLAG_MASK)); // wait if busy
		SPI_PeriControl(SPI2, DISABLE); // disable
		// disable IRQ to receive spi2 interrupts on NVIC
		SPI_IRQInterruptConfig(SPI2_IRQ,DISABLE);

		printf("Received buffer: %s\n", RcvBuff);

		dataAvailable = 0;
		GPIO_IRQInterruptConfig(EXTI15_10, ENABLE); // enable interrupts on pd6 since tx is complete
	}

	return 0;
}


/* Slave data available interrupt handler */
void EXTI15_10_IRQHandler(void)
{
	GPIO_IRQHandling(GPIO_PIN_NO_11);
	dataAvailable = 1;
}

/* Runs when a data byte is received from the peripheral over SPI*/
void SPI2_IRQHandler(void)
{
	SPI_IRQHandling(&SPI2Handle);
}

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{
	static uint32_t i = 0;
	//printf("%c\n", readByte);
	if (AppEv == SPI_EVENT_RX_CMPT)
	{
		RcvBuff[i++] = readByte;
		if (readByte == '\0' || (i == MAX_LEN))
		{
			rcvStop = 1;
			RcvBuff[i-1] = '\0';
			i = 0;
		}
	}
}
