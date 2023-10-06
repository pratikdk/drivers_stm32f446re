/*
 * 005_spi_tx_testing.c
 *
 *  Created on: Sep 20, 2023
 *      Author: prati
 */

#include "stm32f446re_gpio_driver.h"
#include "stm32f446re_spi_driver.h"
#include "stm32f446re.h"
#include <string.h>

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

//	// MISO
//	SPI_GPIOPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
//	GPIO_Init(&SPI_GPIOPins);
//
//	// NSS
//	SPI_GPIOPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
//	GPIO_Init(&SPI_GPIOPins);
}

void SPI2_Inits(void)
{
	SPI_Handle_t SPI2Handle;

	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_EN;

	SPI_Init(&SPI2Handle);
}

int main(void)
{
	char user_data[] = "Hello world";

	SPI2_GPIOInits();

	SPI2_Inits(); // initialize()
	SPI_SSIConfig(SPI2, ENABLE); // makes NSS signal high internally and avoids MODF error
	SPI_PeriControl(SPI2, ENABLE); // enable SPE

	SPI_SendData(SPI2, (uint8_t *) user_data, strlen(user_data));

	SPI_PeriControl(SPI2, DISABLE); // enable

	while(1);

	return 0;
}
