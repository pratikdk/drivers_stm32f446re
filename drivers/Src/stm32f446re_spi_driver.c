/*
 * stm32f446re_spi_driver.c
 *
 *  Created on: Sep 19, 2023
 *      Author: prati
 */

#include "stm32f446re_spi_driver.h"
#include <stddef.h>

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_interrupt_handle(SPI_Handle_t *pSPIHandle);

/*
 * 	Peripheral clock setup
 */

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		if (pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if (pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
	}
	else
	{
		if (pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}
		else if (pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}
	}
}

/*
 *  Init and De-init
 */

void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	// Configure SPI_CR1 register

	uint32_t temp = 0;

	// 1. configure device mode
	temp |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	// 2. configure bus config
	if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		temp &= ~(1 << SPI_CR1_BIDIMODE); // BIDIMODE clear // BIDIMODE set 0
	}
	else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		temp |= (1 << SPI_CR1_BIDIMODE); // BIDIMODE set 1
	}
	else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		temp &= ~(1 << SPI_CR1_BIDIMODE); // BIDIMODE set 0
		temp |= (1 << SPI_CR1_RXONLY); // RXONLY set 1
	}

	// 3. configure sclkspeed (baudrate)
	temp |= (pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR);

	// 4. configure dff
	temp |= (pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF);

	// 5. configure cpol
	temp |= (pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL);

	// 6. configure cpha
	temp |= (pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA);

	// 7. configure ssm
	temp |= (pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM);

	// write temp to CR1 register
	pSPIHandle->pSPIx->CR1 &= ~temp; // clear
	pSPIHandle->pSPIx->CR1 |= temp; // write
}

void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if (pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}
	else if (pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}
	else if (pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}
}

/*
 * Get bit value of a flag in SR register
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagMask)
{
	if (pSPIx->SR & FlagMask)
	{
		return FLAG_SET;
	}

	return FLAG_RESET;
}

/*
 * 	Data send and receive
 */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len)
{
	while (len > 0)
	{
		// 1. Check for TxBuffer to become empty
		while(SPI_GetFlagStatus(pSPIx, SPI_SR_TXE_FLAG_MASK) == FLAG_RESET);

		// 2. Check if 8bit or 16bit
		if (pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			// 16 bit
			// copy 16 bits into data register
			pSPIx->DR = *((uint16_t *) pTxBuffer);
			len--;
			len--;
			(uint16_t *)pTxBuffer++; // point to next 2 bytes
		}
		else
		{
			// 8 bit
			// copy 8 bits into data register
			pSPIx->DR = *pTxBuffer;
			len--;
			pTxBuffer++;
		}
	}
}

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t* pRxBuffer, uint32_t len)
{
	while (len > 0)
	{
		// 1. Check for RxBuffer to become full
		while(SPI_GetFlagStatus(pSPIx, SPI_SR_RXNE_FLAG_MASK) == FLAG_RESET);

		// 2. Check if 8bit or 16bit
		if (pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			// 16 bit
			// copy 16 bits into data register
			*((uint16_t *) pRxBuffer) = pSPIx->DR;
			len--;
			len--;
			(uint16_t *)pRxBuffer++; // point to next 2 bytes
		}
		else
		{
			// 8 bit
			// copy 8 bits into data register
			*pRxBuffer = pSPIx->DR;
			len--;
			pRxBuffer++;
		}
	}
}


/*
 * Send and Receive (Non blocking) data using interrupt (enable interrupt trigger bit)
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t* pTxBuffer, uint32_t len)
{
	uint8_t state = pSPIHandle->TxState;

	// Generate transmission interrupt if not busy in tx
	if (state != SPI_BUSY_IN_TX)
	{
		// 1. save tx buffer and len
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = len;

		// 2. update tx state as busy in tx
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		// 3. generate TXEIE interrupt
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
	}

	return state;
}

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t* pRxBuffer, uint32_t len)
{
	uint8_t state = pSPIHandle->RxState;

	// Generate transmission interrupt if not busy in rx
	if (state != SPI_BUSY_IN_RX)
	{
		// 1. save rx buffer and len
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = len;

		// 2. update rx state as busy in rx
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		// 3. generate rxneie interrupt
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
	}

	return state;
}


/*
 * IRQ Configuration and ISR Handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
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

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprxSection = IRQNumber % 4;
	uint8_t shiftAmount = (iprxSection * 8) + (8 - NUM_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDRESS + (iprx)) |= (IRQPriority << shiftAmount);
}

void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp1, temp2;

	// 1. TXE flag
	temp1 = (pSPIHandle->pSPIx->SR & (1 << SPI_SR_TXE));
	temp2 = (pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE));

	if (temp1 && temp2)
	{
		// handle txe
		spi_txe_interrupt_handle(pSPIHandle);
	}

	// 2. RXNE flag
	temp1 = (pSPIHandle->pSPIx->SR & (1 << SPI_SR_RXNE));
	temp2 = (pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE));

	if (temp1 && temp2)
	{
		// handle rxne
		spi_rxne_interrupt_handle(pSPIHandle);
	}

	// 3. OVR flag
	temp1 = (pSPIHandle->pSPIx->SR & (1 << SPI_SR_OVR));
	temp2 = (pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE));

	if (temp1 && temp2)
	{
		// handle ovr
		spi_ovr_interrupt_handle(pSPIHandle);
	}
}

/*
 * SPI peripheral control; SPE bit
 */
void SPI_PeriControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

/*
 * SSI Config
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

/*
 * SSOE Config
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}
	else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}


/*
 * Interrupt helper function definitions
 */

// IRQ handling

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	// 1. Check if 8bit or 16bit
	if (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
	{
		// 16 bit
		// copy 16 bits into data register
		pSPIHandle->pSPIx->DR = *((uint16_t *) pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t *)pSPIHandle->pTxBuffer++; // point to next 2 bytes
	}
	else
	{
		// 8 bit
		// copy 8 bits into data register
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}

	// 2. Reset state and transmission once tx is complete
	if (!pSPIHandle->TxLen)
	{
		// TxLen is 0; close transmission and inform application
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPT);
	}
}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	// 1. Check if 8bit or 16bit
	if (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
	{
		// 16 bit
		// copy 16 bits from data register
		*((uint16_t *) pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->RxLen--;
		(uint16_t *)pSPIHandle->pRxBuffer++; // point to next 2 bytes
	}
	else
	{
		// 8 bit
		// copy 8 bits from data register
		*pSPIHandle->pRxBuffer = pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}

	// 2. Reset state and reception once rx is complete
	if (!pSPIHandle->RxLen)
	{
		// RxLen is 0; close reception and inform application
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPT);
	}
}

static void spi_ovr_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;

	// 1. clear the ovr flag
	if (pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR; // read
		temp = pSPIHandle->pSPIx->SR; // read
	}
	(void) temp; // unused

	// 2. inform the application
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}


/*
 * helper apis implementation
 */

void SPI_ClearOVRFLag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;

	// 1. clear the ovr flag
	temp = pSPIx->DR; // read
	temp = pSPIx->SR; // read

	(void) temp; // unused
}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->TxState = SPI_READY;
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->RxState = SPI_READY;
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
}

/*
 * Application callback
 */

__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEvt)
{
	// weak implementation, application can override
}
