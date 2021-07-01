/*
 * bootloader_dma.h
 *
 *  Created on: Jun 29, 2021
 *      Author: ziadyasser
 */

#ifndef BOOTLOADER_INC_BOOTLOADER_DMA_H_
#define BOOTLOADER_INC_BOOTLOADER_DMA_H_

#include "stm32f4xx_dma.h"
#include "stm32f4xx_usart.h"
#include "bootloader_cfg.h"

bool BootLoader_DMATransmitError,
	 BootLoader_DMAReceiveError;

void BootLoader_DMAInit(void* DMATransmitBuffer, void* DMAReceiveBuffer, uint32_t DMABufferSize);
void BootLoader_DMAReceiveEnable(void);
void BootLoader_DMAReceiveDisable(void);
void BootLoader_DMATransmitEnable(uint16_t DataLength);
int32_t BootLoader_DMAGetReceivedDataCount(void);
bool BootLoader_DMAIsDataTransmitted(void);
bool BootLoader_DMAIsDataReceived(void);

#endif /* BOOTLOADER_INC_BOOTLOADER_DMA_H_ */
