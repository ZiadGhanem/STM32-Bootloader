/*
 * bootloader_dma.c
 *
 *  Created on: Jun 29, 2021
 *      Author: ziadyasser
 */


#include "bootloader_dma.h"

static int32_t BootLoader_DMABufferSize,
			   BootLoader_DMAReceivedDataCount;

static bool BootLoader_DMADataTransmitted,
			BootLoader_DMADataReceived;

void DMA2_Stream5_IRQHandler(void)
{
	/* Transfer complete interrupt */
	if(DMA_GetITStatus(DMA2_Stream5, DMA_IT_TCIF5))
	{
		BootLoader_DMAReceivedDataCount = BootLoader_DMABufferSize - (int32_t)DMA_GetCurrDataCounter(DMA2_Stream5);
		BootLoader_DMADataReceived = 1;
	}
	/* Transfer error interrupt */
	else if(DMA_GetITStatus(DMA2_Stream5, DMA_IT_TEIF5))
	{
		BootLoader_DMAReceiveError = 1;
	}
	else
	{

	}

	/* Clear the interrupt flags */
	DMA_ClearITPendingBit(DMA2_Stream5, DMA_IT_TCIF5 | DMA_IT_TEIF5);
	/* Enable DMA reception */
	BootLoader_DMAReceiveEnable();
}


void DMA2_Stream7_IRQHandler(void)
{
	/* Transfer complete interrupt */
	if(DMA_GetITStatus(DMA2_Stream7, DMA_IT_TCIF7))
	{
		BootLoader_DMADataTransmitted = 1;
	}
	/* Transfer error interrupt */
	else if(DMA_GetITStatus(DMA2_Stream7, DMA_IT_TEIF7))
	{
		BootLoader_DMATransmitError = 1;
	}
	else
	{

	}

	/* Clear the interrupt flags */
	DMA_ClearITPendingBit(DMA2_Stream7, DMA_IT_TCIF7 | DMA_IT_TEIF7);
}

void BootLoader_DMAInit(void* DMATransmitBuffer, void* DMAReceiveBuffer, uint32_t DMABufferSize)
{
	ASSERT(DMATransmitBuffer != NULL);
	ASSERT(DMAReceiveBuffer != NULL);
	ASSERT(DMABufferSize != 0);
	BootLoader_DMABufferSize = DMABufferSize;
	/* Enable DMA Clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	/* Create the DMA initialization structure */
	DMA_InitTypeDef DMA_InitStruct;
	/******************** DMA reception initialization ********************/
	/* Reset the DMA initialization structure */
	DMA_StructInit(&DMA_InitStruct);
	/* Initialize the DMA initialization structure */
	DMA_InitStruct.DMA_BufferSize = DMABufferSize;
	DMA_InitStruct.DMA_Channel = DMA_Channel_4;
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)DMAReceiveBuffer;
	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_INC4;
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh;
	/* Initialize the DMA */
	DMA_Init(DMA2_Stream5, &DMA_InitStruct);
	/******************** DMA transmission initialization ********************/
	/* Reset the DMA initialization structure */
	DMA_StructInit(&DMA_InitStruct);
	DMA_InitStruct.DMA_BufferSize = DMABufferSize;
	DMA_InitStruct.DMA_Channel = DMA_Channel_4;
	DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)DMATransmitBuffer;
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh;
	/* Initialize the DMA */
	DMA_Init(DMA2_Stream7, &DMA_InitStruct);
	/* Configure the DMA interrupts */
	DMA_ITConfig(DMA2_Stream5, DMA_IT_TC | DMA_IT_TE, ENABLE);
	DMA_ITConfig(DMA2_Stream7, DMA_IT_TC | DMA_IT_TE, ENABLE);
	/******************** NVIC initialization ********************/
	NVIC_SetPriority(DMA2_Stream5_IRQn, 2);
	NVIC_EnableIRQ(DMA2_Stream5_IRQn);
	NVIC_SetPriority(DMA2_Stream7_IRQn, 2);
	NVIC_EnableIRQ(DMA2_Stream7_IRQn);
}

void BootLoader_DMAReceiveEnable(void)
{
	/* Set the number of data units to be received */
	DMA_SetCurrDataCounter(DMA2_Stream5, (uint16_t)BootLoader_DMABufferSize);
	/* Enable reception stream */
	DMA_Cmd(DMA2_Stream5, ENABLE);
}

void BootLoader_DMAReceiveDisable(void)
{
	/* Disable reception stream */
	DMA_Cmd(DMA2_Stream5, DISABLE);
}


void BootLoader_DMATransmitEnable(uint16_t DataLength)
{
	/* Set the number of data units to be transmitted */
	DMA_SetCurrDataCounter(DMA2_Stream7, DataLength);
	/* Enable transmission stream */
	DMA_Cmd(DMA2_Stream7, ENABLE);
}

int32_t BootLoader_DMAGetReceivedDataCount(void)
{
	return BootLoader_DMAReceivedDataCount;
}

bool BootLoader_DMAIsDataTransmitted(void)
{
	if(BootLoader_DMADataTransmitted)
	{
		BootLoader_DMADataTransmitted = false;
		return true;
	}
	else
	{
		return false;
	}
}

bool BootLoader_DMAIsDataReceived(void)
{
	if(BootLoader_DMADataReceived)
	{
		BootLoader_DMADataReceived = false;
		return true;
	}
	else
	{
		return false;
	}
}

