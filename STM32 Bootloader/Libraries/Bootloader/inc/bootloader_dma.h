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

uint8_t BootLoader_DataTransmitted,
		BootLoader_TransmitError,
		BootLoader_DataReceived,
		BootLoader_ReceiveError;

void BootLoader_DMAInit(uint32_t* DMABuffer, uint32_t DMABufferSize);
void BootLoader_DMAReceiveEnable(void);
void BootLoader_DMAReceiveDisable(void);
void BootLoader_DMATransmitEnable(uint16_t DataLength);
int32_t BootLoader_DMAGetReceivedDataCount(void);

#endif /* BOOTLOADER_INC_BOOTLOADER_DMA_H_ */
