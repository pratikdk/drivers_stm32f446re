/*
 * 008_spi_cmd_handling.c
 *
 *  Created on: Sep 26, 2023
 *      Author: prati
 */


#include "stm32f446re_gpio_driver.h"
#include "stm32f446re_spi_driver.h"
#include "stm32f446re.h"
#include <string.h>
#include <stdio.h>

/*
 * commands to exchange between master and slave
 */

#define COMMAND_LED_CONTROL				0x50
#define COMMAND_SENSOR_READ				0x51
#define COMMAND_LED_READ				0x52
#define COMMAND_PRINT					0x53
#define COMMAND_ID_READ					0x54

/*
 * led state
 */

#define LED_ON							1
#define LED_OFF							0

/*
 * arduino analog pins
 */

#define ANALOG_PIN0						0
#define ANALOG_PIN1						1
#define ANALOG_PIN2						2
#define ANALOG_PIN3						3
#define ANALOG_PIN4						4

/*
 * Arduino led
 */

#define LED_PIN							9

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
	SPI_Handle_t SPI2Handle;

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

void GPIO_ButtonInit()
{
	GPIO_Handle_t GpioBtn;

	// blue button config
	GpioBtn.pGPIOx = GPIOC;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NO;

	// Initialize GPIOC peripheral
	GPIO_Init(&GpioBtn);
}

uint8_t SPI_VerifyResponse(uint8_t ackByte)
{
	if (ackByte == 0xF5)
	{
		// ack
		return 1;
	}
	// nack
	return 0;
}

int main(void)
{
	initialise_monitor_handles();

	uint8_t dummy_write = 0;
	uint8_t dummy_read = 0;

	GPIO_ButtonInit();

	SPI2_GPIOInits();

	SPI2_Inits(); // initialize()

	SPI_SSOEConfig(SPI2, ENABLE);

	while (1)
	{

		while (GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));
		delay();

		printf("SPI communication started\n");
		//SPI_SSIConfig(SPI2, ENABLE); // makes NSS signal high internally and avoids MODF error
		SPI_PeriControl(SPI2, ENABLE); // enable

//		// send length info
//		uint8_t dataLen = strlen(user_data);
//		SPI_SendData(SPI2, &dataLen, 1);
//
//		// send data
//		SPI_SendData(SPI2, (uint8_t *) user_data, strlen(user_data));

		uint8_t command_code;
		uint8_t ackbyte;
		uint8_t args[2];

		// 1. COMMAND_LED_CONTROL
		command_code = COMMAND_LED_CONTROL;
		SPI_SendData(SPI2, &command_code, 1); // send
		SPI_ReceiveData(SPI2, &dummy_read, 1); // fetch; clear off rxne bit
		// fetch ack/nack response
		SPI_SendData(SPI2, &dummy_write, 1); // send
		SPI_ReceiveData(SPI2, &ackbyte, 1); // fetch

		if (SPI_VerifyResponse(ackbyte))
		{
			args[0] = LED_PIN;
			args[1] = LED_ON;
			SPI_SendData(SPI2, args, 2); // send
			SPI_ReceiveData(SPI2, &dummy_read, 1); // fetch; clear off rxne bit

			printf("COMMAND_LED_CONTROL: finished\n");
		}


		// 2. COMMAND_SENSOR_READ
		while (GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));
		delay();

		command_code = COMMAND_SENSOR_READ;
		SPI_SendData(SPI2, &command_code, 1); // send
		SPI_ReceiveData(SPI2, &dummy_read, 1); // fetch; clear off rxne bit

		// fetch ack/nack response
		SPI_SendData(SPI2, &dummy_write, 1); // send
		SPI_ReceiveData(SPI2, &ackbyte, 1); // fetch

		if (SPI_VerifyResponse(ackbyte))
		{
			args[0] = ANALOG_PIN0;
			SPI_SendData(SPI2, args, 1); // send
			SPI_ReceiveData(SPI2, &dummy_read, 1); // fetch; clear off rxne bit

			// delay, slave does adc conversion, hence wait for analog value to converted by slave
			delay();

			// fetch the response from slave
			uint8_t analog_read;
			SPI_SendData(SPI2, &dummy_write, 1); // send dummy byte
			SPI_ReceiveData(SPI2, &analog_read, 1); // fetch response byte

			printf("COMMAND_SENSOR_READ: sensor value = %d\n", analog_read);
		}


		// 3. COMMAND_LED_READ
		while (GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));
		delay();

		command_code = COMMAND_LED_READ;
		SPI_SendData(SPI2, &command_code, 1); // send
		SPI_ReceiveData(SPI2, &dummy_read, 1); // fetch; clear off rxne bit

		// fetch ack/nack response
		SPI_SendData(SPI2, &dummy_write, 1); // send
		SPI_ReceiveData(SPI2, &ackbyte, 1); // fetch

		if (SPI_VerifyResponse(ackbyte))
		{
			args[0] = LED_PIN;
			SPI_SendData(SPI2, args, 1); // send
			SPI_ReceiveData(SPI2, &dummy_read, 1); // fetch; clear off rxne bit

			// fetch the response from slave
			uint8_t pin_read;
			SPI_SendData(SPI2, &dummy_write, 1); // send dummy byte
			SPI_ReceiveData(SPI2, &pin_read, 1); // fetch response byte

			printf("COMMAND_LED_READ: pin value = %d\n", pin_read);
		}


		// 4. COMMAND_PRINT
		while (GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));
		delay();

		command_code = COMMAND_PRINT;
		SPI_SendData(SPI2, &command_code, 1); // send
		SPI_ReceiveData(SPI2, &dummy_read, 1); // fetch; clear off rxne bit

		// fetch ack/nack response
		SPI_SendData(SPI2, &dummy_write, 1); // send
		SPI_ReceiveData(SPI2, &ackbyte, 1); // fetch

		uint8_t message[] = "Hello! how are you";

		if (SPI_VerifyResponse(ackbyte))
		{
			args[0] = strlen((char *) message);

			SPI_SendData(SPI2, args, 1); // send
			SPI_ReceiveData(SPI2, &dummy_read, 1); // fetch; clear off rxne bit

			SPI_SendData(SPI2, message, args[0]); // send
			SPI_ReceiveData(SPI2, &dummy_read, 1); // fetch; clear off rxne bit

			printf("COMMAND_PRINT: finished\n");
		}


		// 5. COMMAND_ID_READ
		while (GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));
		delay();

		command_code = COMMAND_ID_READ;
		SPI_SendData(SPI2, &command_code, 1); // send
		SPI_ReceiveData(SPI2, &dummy_read, 1); // fetch; clear off rxne bit

		// fetch ack/nack response
		SPI_SendData(SPI2, &dummy_write, 1); // send
		SPI_ReceiveData(SPI2, &ackbyte, 1); // fetch

		uint8_t id[11];
		uint32_t i;
		if (SPI_VerifyResponse(ackbyte))
		{
			for (i = 0; i < 10; ++i)
			{
				SPI_SendData(SPI2, &dummy_write, 1); // send
				SPI_ReceiveData(SPI2, &id[i], 1); // fetch; clear off rxne bit
			}

			id[10] = '\0';

			printf("COMMAND_ID_READ: %s\n", id);
		}


		// Disable SPI2 peripheral

		while (SPI_GetFlagStatus(SPI2, SPI_SR_BSY_FLAG_MASK));
		SPI_PeriControl(SPI2, DISABLE); // enable

		printf("SPI communication closed\n");
	}

	return 0;
}
