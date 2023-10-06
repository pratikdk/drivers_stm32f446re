/*
 * stm32f446re_i2c_driver.c
 *
 *  Created on: Sep 28, 2023
 *      Author: prati
 */


#include "stm32f446re_i2c_driver.h"
#include <stddef.h>


/*
 * 	Peripheral clock setup
 */

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		if (pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}
		else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}
		else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}
	else
	{
		if (pI2Cx == I2C1)
		{
			I2C1_PCLK_DI();
		}
		else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_DI();
		}
		else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_DI();
		}
	}
}


// Utility for Init and De-init
uint16_t AHB_PreScaler[] = {2, 4, 8, 16, 64, 128, 256, 512};
uint8_t APB1_PreScaler[] = {2, 4, 8, 16};

uint32_t RCC_GetPCLK1Value(void)
{
	// (SYSCLK value / AHB Prescaler value) / APB1 Prescaler value
	uint32_t pclk1;

	// 1. get SYSCLK
	uint8_t sysclkSource;
	sysclkSource = ((RCC->CFGR >> 2) & 0x3);

	uint32_t syslClk;
	if (sysclkSource == 0)
	{
		// HSI; 16mhz
		syslClk = 16000000;
	}
	else if (sysclkSource == 1)
	{
		// HSE; 8mhz
		syslClk = 8000000;
	}

	// 2. get AHB Prescaler
	uint8_t hpre = ((RCC->CFGR >> 4) & 0xF);
	uint16_t ahbp;

	if (hpre < 8)
	{
		ahbp = 1;
	}
	else
	{
		ahbp = AHB_PreScaler[hpre - 8];
	}

	// 3. get APB1 Prescaler
	uint8_t ppre1 = ((RCC->CFGR >> 10) & 0x7);
	uint16_t apbp1;

	if (ppre1 < 4)
	{
		apbp1 = 1;
	}
	else
	{
		apbp1 = APB1_PreScaler[ppre1 - 4];
	}

	// compute pclk1
	pclk1 = ((syslClk / ahbp) / apbp1);

	return pclk1;
}

/*
 *  Init and De-init
 */

void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t temp = 0;

	// Enable peripheral clock
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	// 1. Configure SCL by setting ACK_Control(CR1) and FREQ(CR2) bits
	//temp = (pI2CHandle->I2C_Config.I2C_ACKControl << I2C_CR1_ACK);
	temp |= pI2CHandle->I2C_Config.I2C_ACKControl << 10;
	pI2CHandle->pI2Cx->CR1 |= temp;

	temp = RCC_GetPCLK1Value() / 1000000U;
	pI2CHandle->pI2Cx->CR2 |= (temp & 0x3F); // 5 bits

	// 2. Configure device own address
	temp = (pI2CHandle->I2C_Config.I2C_DeviceAddress << 1); // 7 bit address + 1
	temp |= (1 << 14); // rm specifies should be set as 1
	pI2CHandle->pI2Cx->OAR1 |= temp;

	// 3. Configure CCR to set the user selected clock speed
	uint16_t ccrValue;
	if (pI2CHandle->I2C_Config.I2C_SCLSpeed  <= I2C_SCL_SPEED_SM)
	{
		// standard mode
		ccrValue = RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		temp = (ccrValue & 0xFFF);
	}
	else
	{
		// fast mode
		temp = (1 << I2C_CCR_FS); // enable fast mode
		temp |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << I2C_CCR_DUTY); // configure duty cycle
		if (pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			// duty tlow = tHigh (hence 50% each)
			ccrValue = RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}
		else
		{
			// duty 16:9 = tlow:thigh
			ccrValue = RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}
		temp |= (ccrValue & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR |= temp;

	// 4. configure I2C_TRISE register (6 bits)
	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		// standard mode
		temp = (RCC_GetPCLK1Value() / 1000000U) + 1;  // compute trise = trise_scl_max / t_pclk1
	}
	else
	{
		// fast mode
		temp = ((RCC_GetPCLK1Value() * 300) / 1000000U) + 1;  // compute trise
	}
	pI2CHandle->pI2Cx->TRISE = (temp & 0x3F);
}

void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	if (pI2Cx == I2C1)
	{
		I2C1_REG_RESET();
	}
	else if (pI2Cx == I2C2)
	{
		I2C2_REG_RESET();
	}
	else if (pI2Cx == I2C3)
	{
		I2C3_REG_RESET();
	}
}

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr)
{
	slaveAddr = slaveAddr << 1;
	slaveAddr &= ~(1); //SlaveAddr is Slave address + r/nw bit=0
	pI2Cx->DR = slaveAddr;
}

static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr)
{
	slaveAddr = slaveAddr << 1;
	slaveAddr |= (1); //SlaveAddr is Slave address + r/nw bit=1
	pI2Cx->DR = slaveAddr;
}

static void I2C_ClearAddrFlag(I2C_RegDef_t *pI2Cx)
{
	uint32_t dummyRead = pI2Cx->SR1;
	dummyRead = pI2Cx->SR2;
	(void) dummyRead;
}

static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

static void I2C_ManagaAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if (EnorDi == I2C_ACK_ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	}
	else
	{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
	}
}

/*
 * Data send and receive
 */

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t slaveAddress)
{
	// 1. Generate the start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// 2. confirm that start generation is completed by checking the sb flag in the SR1
	// Until SB is cleared SCL will be stretched (pulled to low)
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	// 3. send the address of the slave with r/w bit set to w(0) (total 8 bits)
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, slaveAddress);

	// 4. confirm that address phase is completed by checking the addr flag in the SR1
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	// 5. clear the addr flag
	// Until ADDR is cleared SCL will be stretched (pulled to low)
	I2C_ClearAddrFlag(pI2CHandle->pI2Cx);

	// 6. send the data until len becomes 0
	while (len > 0)
	{
		while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TxE));
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		len--;
	}

	// 7. when len becomes 0 wait for txe=1 and btf=1 before generating the stop condition
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TxE));
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF));

	// 8. generate stop condition
	// generating stop, automatically clears the btf
	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
}

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t len, uint8_t slaveAddress)
{
	// 1. Generate the start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// 2. confirm that start generation is completed by checking the sb flag in the SR1
	// Until SB is cleared SCL will be stretched (pulled to low)
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	// 3. send the address of the slave with r/w bit set to r(1) (total 8 bits)
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, slaveAddress);

	// 4. confirm that address phase is completed by checking the addr flag in the SR1
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));


	// Read a byte from slave
	if (len == 1)
	{
		// Disable acking
		I2C_ManagaAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

		// generate stop condition
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		// clear the addr flag
		I2C_ClearAddrFlag(pI2CHandle->pI2Cx);

		// wait until rxne becomes 1
		while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RxNE));

		// read data from data register into buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;

		return;
	}

	// Read data from slave when len > 1
	if (len > 1)
	{
		// clear the addr flag
		I2C_ClearAddrFlag(pI2CHandle->pI2Cx);

		// read data until length becomes zero
		for (uint32_t i = len; i > 0; --i)
		{
			// wait until rxne becomes 1
			while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RxNE));

			// if last 2 bytes are remaining
			if (i == 2)
			{
				// Disable acking
				I2C_ManagaAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

				// generate stop condition
				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			}

			// read data from data register into buffer
			*pRxBuffer = pI2CHandle->pI2Cx->DR;

			// increment the buffer address
			pRxBuffer++;
		}
	}

	// re-enable acking
	if (pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		I2C_ManagaAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
	}
}

/*
 * IRQ Configuration and ISR Handling
 */

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		if (IRQNumber < 32)
		{
			*NVIC_ISER0 |= (1 << (IRQNumber % 32));
		}
		else if (IRQNumber >= 32 && IRQNumber < 64)
		{
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if (IRQNumber >= 64 && IRQNumber < 96)
		{
			*NVIC_ISER2 |= (1 << (IRQNumber % 32));
		}
		else if (IRQNumber >= 96 && IRQNumber < 128)
		{
			*NVIC_ISER3 |= (1 << (IRQNumber % 32));
		}
	}
	else
	{
		if (IRQNumber < 32)
		{
			*NVIC_ICER0 |= (1 << (IRQNumber % 32));
		}
		else if (IRQNumber >= 32 && IRQNumber < 64)
		{
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		else if (IRQNumber >= 64 && IRQNumber < 96)
		{
			*NVIC_ICER2 |= (1 << (IRQNumber % 32));
		}
		else if (IRQNumber >= 96 && IRQNumber < 128)
		{
			*NVIC_ICER3 |= (1 << (IRQNumber % 32));
		}
	}
}

void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprxSection = IRQNumber % 4;
	uint8_t shiftAmount = (iprxSection * 8) + (8 - NUM_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDRESS + (iprx)) |= (IRQPriority << shiftAmount);
}

/*
 * Get bit value of a flag in SRx register
 */

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
	if (pI2Cx->SR1 & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}


/*
 * I2C peripheral control; PE bit
 */

void I2C_PeriControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	}
	else
	{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}

/*
 * Application callback
 */

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEvt)
{

}
