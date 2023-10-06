/*
 * stm32f446re_i2c_driver.h
 *
 *  Created on: Sep 28, 2023
 *      Author: prati
 */

#ifndef INC_STM32F446RE_I2C_DRIVER_H_
#define INC_STM32F446RE_I2C_DRIVER_H_

#include "stm32f446re.h"

/*
 * Configuration structure for I2Cx peripheral
 */
typedef struct
{
	uint32_t I2C_SCLSpeed;
	uint8_t	I2C_DeviceAddress;
	uint32_t I2C_ACKControl;
	uint8_t I2C_FMDutyCycle;
} I2C_Config_t;

/*
 * Handle structure for I2Cx peripheral
 */

typedef struct
{
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_Config;
} I2C_Handle_t;


/*
 * I2C_SCLKSpeed values
 */

#define I2C_SCL_SPEED_SM				100000
#define I2C_SCL_SPEED_FM4k				400000
#define I2C_SCL_SPEED_FM2k				200000

/*
 * I2C_ACKControl values
 */

#define I2C_ACK_ENABLE					1
#define I2C_ACK_DISABLE					0

/*
 * I2C_FMDutyCycle values
 */

#define I2C_FM_DUTY_2					0
#define I2C_FM_DUTY_16_9				1


/*
 * SPI Related Status Flag mask macros
 */

#define I2C_FLAG_SB						(1 << I2C_SR1_SB)
#define I2C_FLAG_ADDR					(1 << I2C_SR1_ADDR)
#define I2C_FLAG_BTF					(1 << I2C_SR1_BTF)
#define I2C_FLAG_ADD10					(1 << I2C_SR1_ADD10)
#define I2C_FLAG_STOPF					(1 << I2C_SR1_STOPF)
#define I2C_FLAG_RxNE					(1 << I2C_SR1_RxNE)
#define I2C_FLAG_TxE					(1 << I2C_SR1_TxE)
#define I2C_FLAG_BERR					(1 << I2C_SR1_BERR)
#define I2C_FLAG_ARLO					(1 << I2C_SR1_ARLO)
#define I2C_FLAG_AF						(1 << I2C_SR1_AF)
#define I2C_FLAG_OVR					(1 << I2C_SR1_OVR)
#define I2C_FLAG_PECERR					(1 << I2C_SR1_PECERR)
#define I2C_FLAG_TIMEOUT				(1 << I2C_SR1_TIMEOUT)
#define I2C_FLAG_SMBALERT				(1 << I2C_SR1_SMBALERT)


/*
 * 	Peripheral clock setup
 */

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);

/*
 *  Init and De-init
 */

void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/*
 * Data send and receive
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t slaveAddress);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t len, uint8_t slaveAddress);

/*
 * IRQ Configuration and ISR Handling
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

/*
 * Get bit value of a flag in SRx register
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagMask);

/*
 * I2C peripheral control; PE bit
 */
void I2C_PeriControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);

/*
 * Application callback
 */

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEvt);


#endif /* INC_STM32F446RE_I2C_DRIVER_H_ */
