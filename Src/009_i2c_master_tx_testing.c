/*
 * 009_i2c_master_tx_testing.c
 *
 *  Created on: Oct 3, 2023
 *      Author: prati
 */

#include "stm32f446re_gpio_driver.h"
#include "stm32f446re_i2c_driver.h"
#include "stm32f446re.h"
#include <string.h>
#include <stdio.h>

/*
 * PB6 -> SCL
 * PB7 -> SDA
 */
#define MASTER_PLACEHOLDER_ADDR 	0x62
#define SLAVE_ADDR					0x68
I2C_Handle_t I2C1Handle;

uint8_t some_data[] = "We are testing I2C master Tx\n";

void delay(void)
{
	for (uint32_t i = 0; i < 500000/2; ++i);
}

void I2C_GPIOInits(void)
{
	GPIO_Handle_t I2CPins;

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NO;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// SCL
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	GPIO_Init(&I2CPins);

	// SDA
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&I2CPins);

}

void I2C_Inits(void)
{
	I2C1Handle.pI2Cx = I2C1;

	I2C1Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MASTER_PLACEHOLDER_ADDR;

	I2C_Init(&I2C1Handle);
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


int main()
{
	// GPIOC pin 13; onboard blue button
	GPIO_ButtonInit();

	// I2C pin inits
	I2C_GPIOInits();

	// I2C peripheral configuration
	I2C_Inits();

	// Enable I2C peripheral
	I2C_PeriControl(I2C1Handle.pI2Cx, ENABLE);

	I2C1Handle.pI2Cx->CR1 |= (I2C1Handle.I2C_Config.I2C_ACKControl << 10);
	while (1)
	{
		// wait for button press
		while (GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));
		delay(); // de-bouncing

		// send some data to the slave
		I2C_MasterSendData(&I2C1Handle, some_data, strlen((char *) some_data), SLAVE_ADDR);
	}
	return 0;
}
