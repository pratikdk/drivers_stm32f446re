/*
 * stm32f446re_usart_driver.c
 *
 *  Created on: Oct 5, 2023
 *      Author: prati
 */

#include "stm32f446re_usart_driver.h"
#include <stddef.h>

/*
 * Peripheral Clock setup
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (pUSARTx == USART1)
		{
			USART1_PCLK_EN();
		}
		else if (pUSARTx == USART2)
		{
			USART2_PCLK_EN();
		}
		else if (pUSARTx == USART3)
		{
			USART3_PCLK_EN();
		}
		else if (pUSARTx == UART4)
		{
			UART4_PCLK_EN();
		}
		else if (pUSARTx == UART5)
		{
			UART5_PCLK_EN();
		}
		else if (pUSARTx == USART6)
		{
			USART6_PCLK_EN();
		}
	}
	else
	{
		if (pUSARTx == USART1)
		{
			USART1_PCLK_DI();
		}
		else if (pUSARTx == USART2)
		{
			USART2_PCLK_DI();
		}
		else if (pUSARTx == USART3)
		{
			USART3_PCLK_DI();
		}
		else if (pUSARTx == UART4)
		{
			UART4_PCLK_DI();
		}
		else if (pUSARTx == UART5)
		{
			UART5_PCLK_DI();
		}
		else if (pUSARTx == USART6)
		{
			USART6_PCLK_DI();
		}
	}
}

/*
 * Init and De-init
 */
void USART_Init(USART_Handle_t *pUSARTHandle)
{
	uint32_t temp = 0;

	/********** Configuration of CR1 **************/
	// Enable peripheral clock
	USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);

	// Enable Tx and/or Rx engines
	if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
	{
		temp |= (1 << USART_CR1_TE);
	}
	else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
	{
		temp |= (1 << USART_CR1_TE);
	}
	else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX)
	{
		temp |= (1 << USART_CR1_TE);
		temp |= (1 << USART_CR1_TE);
	}

	// configure word length
	temp |= (pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M);

    // configuration of parity control bit fields
	if ( pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
	{
		// enable the parity control
		temp |= ( 1 << USART_CR1_PCE);

		// enable EVEN parity
		// Not required because by default EVEN parity will be selected
		temp &= ~( 1 << USART_CR1_PS);

	}else if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD )
	{
		// enable the parity control
		temp |= ( 1 << USART_CR1_PCE);

	    // enable ODD parity
		temp |= ( 1 << USART_CR1_PS);
	}

	// Program the CR1 register
	pUSARTHandle->pUSARTx->CR1 = temp;

	/************ Configuration of CR2 *************/
	temp = 0;

	// configure the number of stop bits inserted during USART frame transmission
	temp |= pUSARTHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP;

	// Program the CR2 register
	pUSARTHandle->pUSARTx->CR2 = temp;

	/************ Configuration of CR3 ************/

	temp = 0;

	// Configuration of USART hardware flow control
	if ( pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
	{
		// enable CTS flow control
		temp |= ( 1 << USART_CR3_CTSE);


	}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
	{
		// enable RTS flow control
		temp |= ( 1 << USART_CR3_RTSE);

	}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
	{
		// enable both CTS and RTS Flow control
		temp |= ( 1 << USART_CR3_CTSE);
		temp |= ( 1 << USART_CR3_RTSE);
	}


	pUSARTHandle->pUSARTx->CR3 = temp;

	/************  Configuration of BRR(Baudrate register) ************/
}

void USART_DeInit(USART_RegDef_t *pUSARTx)
{

}


/*
 * Data Send and Receive
 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint16_t *pdata;

	// loop over len number of bytes to transfered
	for (uint32_t i = 0; i < Len; ++i)
	{
		// wait until txe flag is set in the SR
		while (!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TXE));

		// Check the USART_WordLength item for 9BIT or 8BIT in a frame
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			// load dr with 9 bits, while masking other bits
			pdata = (uint16_t *) pTxBuffer;
			pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t) 0x01FF);

			// check for USART_ParityControl
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				// No parity is used in this transfer, so 9bits of user data will be sent
				// increment pTxBuffer twice
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				// Parity bit is used in this transfer . so 8bits of user data will be sent
				// The 9th bit will be replaced by parity bit by the hardware
				pTxBuffer++;
			}
		}
		else
		{
			// This is 8bit data transfer
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer  & (uint8_t) 0xFF);

			// code to increment the buffer address
			pTxBuffer++;
		}
	}

	// wait till TC flag is set in the SR
	while( ! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TC));
}

void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	// Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{
		// wait until RXNE flag is set in the SR
		while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_RXNE));

		//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			// receive 9bit data in a frame

			// check if USART_ParityControl control or not enable
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				// No parity is used , so all 9bits will be of user data

				// read only first 9 bits so mask the DR with 0x01FF
				*((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);

				// increment the pRxBuffer two times
				pRxBuffer++;
				pRxBuffer++;
			}
			else
			{
				// Parity is used, so 8bits will be of user data and 1 bit is parity
				 *pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
				 pRxBuffer++;
			}
		}
		else
		{
			// receive 8bit data in a frame

			// Now, check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				// No parity is used , so all 8bits will be of user data

				// read 8 bits from DR
				 *pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
			}

			else
			{
				// Parity is used, so , 7 bits will be of user data and 1 bit is parity

				// read only 7 bits , hence mask the DR with 0X7F
				 *pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0x7F);
			}

			// increment the pRxBuffer
			pRxBuffer++;
		}
	}
}

uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len)
{
//	uint8_t txstate = pUSARTHandle->TxBusyState;
//
//	if(txstate != USART_BUSY_IN_TX)
//	{
//		pUSARTHandle->TxLen = Len;
//		pUSARTHandle->pTxBuffer = pTxBuffer;
//		pUSARTHandle->TxBusyState = USART_BUSY_IN_TX;
//
//		//Implement the code to enable interrupt for TXE
//		pUSARTHandle->pUSARTx->CR1 |= ( 1 << USART_CR1_TXEIE);
//
//
//		//Implement the code to enable interrupt for TC
//		pUSARTHandle->pUSARTx->CR1 |= ( 1 << USART_CR1_TCIE);
//	}
//	return txstate;
}

uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
//	uint8_t rxstate = pUSARTHandle->RxBusyState;
//
//	if(rxstate != USART_BUSY_IN_RX)
//	{
//		pUSARTHandle->RxLen = Len;
//		pUSARTHandle->pRxBuffer = pRxBuffer;
//		pUSARTHandle->RxBusyState = USART_BUSY_IN_RX;
//
//		(void)pUSARTHandle->pUSARTx->DR;
//
//		//Implement the code to enable interrupt for RXNE
//		pUSARTHandle->pUSARTx->CR1 |= ( 1 << USART_CR1_RXNEIE);
//
//	}
//	return rxstate;
}


/*
 * IRQ Configuration and ISR handling
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
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

void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprxSection = IRQNumber % 4;
	uint8_t shiftAmount = (iprxSection * 8) + (8 - NUM_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDRESS + (iprx)) |= (IRQPriority << shiftAmount);
}

void USART_IRQHandling(USART_Handle_t *pHandle)
{

}


/*
 * Other Peripheral Control APIs
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		pUSARTx->CR1 |= (1 << USART_CR1_UE);
	}
	else
	{
		pUSARTx->CR1 &= ~(1 << USART_CR1_UE);
	}
}

uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx , uint32_t FlagName)
{
	if (pUSARTx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName)
{
	pUSARTx->SR &= ~(StatusFlagName);
}


/*
 * Application callback
 */
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t AppEv)
{

}

