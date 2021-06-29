/*
 * bootloader_usart.c
 *
 *  Created on: Jun 29, 2021
 *      Author: ziadyasser
 */

#include "bootloader_usart.h"

void USART1_IRQHandler(void)
{
	if(USART_GetITStatus(USART1, USART_IT_IDLE))
	{
		/* To clear parity error flag */
		(void)USART_ReceiveData(USART1);
		/* Disable DMA reception */
		BootLoader_DMAReceiveDisable();
	}
	else
	{

	}
}

void BootLoader_USARTInit(uint32_t BootLoader_USARTBaudRate)
{
	/* Enable USART clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	/* Create the USART initialization structure */
	USART_InitTypeDef USART_InitStruct;
	/* Initialize the USART initialization structure */
	USART_InitStruct.USART_BaudRate = BootLoader_USARTBaudRate;
	USART_InitStruct.USART_WordLength = USART_WordLength_9b;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_Parity = USART_Parity_Even;
	USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	/* Initialize the USART */
	USART_Init(USART1, &USART_InitStruct);
	/* Enable USART Interrupts */
	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
	/* Enable the USART DMA */
	USART_DMACmd(USART1, USART_DMAReq_Tx | USART_DMAReq_Rx, ENABLE);
	/******************** NVIC initialization ********************/
	NVIC_SetPriority(USART1_IRQn, 2);
	NVIC_EnableIRQ(USART1_IRQn);
}


void BootLoader_USARTEnable(void)
{
	/* Enable the USART */
	USART_Cmd(USART1, ENABLE);
}
