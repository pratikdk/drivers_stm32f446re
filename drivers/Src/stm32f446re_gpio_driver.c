/*
 * stm32f446re_gpio_driver.c
 *
 *  Created on: Sep 15, 2023
 *      Author: prati
 */


#include "stm32f446re_gpio_driver.h"
#include "stm32f446re.h"

/*
 * 	Peripheral clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
	}
	else
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}
		else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
	}
}


/*
 *  Init and De-init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{

	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	uint32_t temp;

//	1. configure mode register using GPIO_PinMode value for GPIO_PinNumber
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		// non interrupt modes
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER 	&= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER 	|= temp;
	}
	else
	{
		// interrupt modes
		// first, set pin as input mode
		temp = (GPIO_MODE_IN << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER 	&= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER 	|= temp;

		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			// 1. set FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// clear RTSR
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			// 2. set RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// clear FTSR
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			// 1. set RTSR and set FTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		// 2. Configure the GPIO port selection using SYSCFG_EXTICR
		uint8_t temp1, temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint32_t portCode = GPIO_BASEADDRESS_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN(); // enable peripheral clock
		SYSCFG->EXTICR[temp1] |= (portCode << (temp2 * 4));

		// 3. Enable the EXTIx interrupt delivery to NVIC using IMR (unmask the x line)
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

//	2. configure speed register using GPIO_PinSpeed value for GPIO_PinNumber
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDER 	&= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDER 	|= temp;

//	3. configure pull up/pull down control register using GPIO_PinPuPdControl value for GPIO_PinNumber
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR 		&= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR 		|= temp;

//	4. configure output type register using GPIO_PinOPType value for GPIO_PinNumber
//	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode != GPIO_MODE_OUT)
//	{
//		temp = (GPIO_OP_TYPE_PP << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
//	}
//	else
//	{
//		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
//	}
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER 	&= ~(0x1 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER 	|= temp;

//	5. configure alternate function register using GPIO_PinAltFunMode value for GPIO_PinNumber
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint8_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8; // array index
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;	// register pin offset
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (temp2 * 4));
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (temp2 * 4));
		pGPIOHandle->pGPIOx->AFR[temp1] |= temp;
	}
}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if (pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if (pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if (pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if (pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if (pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if (pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}
	else if (pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}
	else if (pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
}


/*
 * 	Data read and write
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t) ((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t) pGPIOx->IDR;
	return value;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if (Value == SET)
	{
		pGPIOx->ODR |= (0x1 << PinNumber);
	}
	else
	{
		pGPIOx->ODR &= ~(0x1 << PinNumber);
	}
}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{

	pGPIOx->ODR = Value;
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}


/*
 * IRQ Configuration and ISR Handling
 */

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
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

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprxSection = IRQNumber % 4;
	uint8_t shiftAmount = (iprxSection * 8) + (8 - NUM_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDRESS + (iprx)) |= (IRQPriority << shiftAmount);
}

void GPIO_IRQHandling(uint8_t PinNumber)
{
	if (EXTI->PR & (1 << PinNumber))
	{
		// clear
		EXTI->PR |= (1 << PinNumber);
	}
}
