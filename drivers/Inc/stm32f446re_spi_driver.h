/*
 * stm32f446re_spi_driver.h
 *
 *  Created on: Sep 19, 2023
 *      Author: prati
 */

#ifndef INC_STM32F446RE_SPI_DRIVER_H_
#define INC_STM32F446RE_SPI_DRIVER_H_

#include "stm32f446re.h"

/*
 * 	A configuration structure for a SPIx peripheral
 */

typedef struct
{
	uint8_t 				SPI_DeviceMode;
	uint8_t 				SPI_BusConfig;
	uint8_t					SPI_SclkSpeed;
	uint8_t					SPI_DFF;
	uint8_t					SPI_CPOL;
	uint8_t					SPI_CPHA;
	uint8_t					SPI_SSM;
} SPI_Config_t;

/*
 * 	A handle structure for a SPIx peripheral
 */

typedef struct
{
	SPI_RegDef_t 					*pSPIx;
	SPI_Config_t					SPIConfig;
	uint8_t							*pTxBuffer;
	uint8_t							*pRxBuffer;
	uint32_t						TxLen;
	uint32_t						RxLen;
	uint8_t							TxState;
	uint8_t							RxState;
} SPI_Handle_t;


/*
 * SPI_DeviceMode;
 */

#define	SPI_DEVICE_MODE_SLAVE			0
#define	SPI_DEVICE_MODE_MASTER			1

/*
 * SPI_BusConfig;
 */

#define	SPI_BUS_CONFIG_FD				1
#define	SPI_BUS_CONFIG_HD				2
#define	SPI_BUS_CONFIG_SIMPLEX_RXONLY	3

/*
 * SPI_SclkSpeed;
 */

#define SPI_SCLK_SPEED_DIV2				0
#define SPI_SCLK_SPEED_DIV4				1
#define SPI_SCLK_SPEED_DIV8				2
#define SPI_SCLK_SPEED_DIV16			3
#define SPI_SCLK_SPEED_DIV32			4
#define SPI_SCLK_SPEED_DIV64			5
#define SPI_SCLK_SPEED_DIV128			6
#define SPI_SCLK_SPEED_DIV256			7

/*
 * SPI_DFF;
 */

#define SPI_DFF_8BITS					0
#define SPI_DFF_16BITS					1

/*
 * SPI_CPOL;
 */

#define SPI_CPOL_LOW					0
#define SPI_CPOL_HIGH					1

/*
 * SPI_CPHA;
 */

#define SPI_CPHA_LOW					0
#define SPI_CPHA_HIGH					1

/*
 * SPI_SSM;
 */

#define SPI_SSM_DI						0
#define SPI_SSM_EN						1

/*
 * State macros
 */

#define SPI_READY						0
#define SPI_BUSY_IN_RX					1
#define SPI_BUSY_IN_TX					2

/*
 * SPI application events
 */

#define SPI_EVENT_TX_CMPT				1
#define SPI_EVENT_RX_CMPT				2
#define SPI_EVENT_OVR_ERR				3
#define SPI_EVENT_CRC_ERR				4

/*
 * SPI Related Status Flag mask macros
 */

#define SPI_SR_RXNE_FLAG_MASK			(1 << SPI_SR_RXNE)
#define SPI_SR_TXE_FLAG_MASK			(1 << SPI_SR_TXE)
#define SPI_SR_CHSIDE_FLAG_MASK			(1 << SPI_SR_CHSIDE)
#define SPI_SR_UDR_FLAG_MASK			(1 << SPI_SR_UDR)
#define SPI_SR_CRCERR_FLAG_MASK			(1 << SPI_SR_CRCERR)
#define SPI_SR_MODF_FLAG_MASK			(1 << SPI_SR_MODF)
#define SPI_SR_OVR_FLAG_MASK			(1 << SPI_SR_OVR)
#define SPI_SR_BSY_FLAG_MASK			(1 << SPI_SR_BSY)
#define SPI_SR_FRE_FLAG_MASK			(1 << SPI_SR_FRE)

/*
 * 	Peripheral clock setup
 */

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

/*
 *  Init and De-init
 */

void SPI_Init(SPI_Handle_t *pSPIHandle);

void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * 	Data send and receive
 */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t* pTxBuffer, uint32_t len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t* pRxBuffer, uint32_t len);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t* pTxBuffer, uint32_t len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t* pRxBuffer, uint32_t len);

/*
 * IRQ Configuration and ISR Handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

/*
 * Get bit value of a flag in SR register
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagMask);

/*
 * SPI peripheral control; SPE bit
 */
void SPI_PeriControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

/*
 * SSI Config
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

/*
 * SSOE Config
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

/*
 * helper apis
 */

void SPI_ClearOVRFLag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

/*
 * Application callback
 */

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEvt);

#endif /* INC_STM32F446RE_SPI_DRIVER_H_ */
