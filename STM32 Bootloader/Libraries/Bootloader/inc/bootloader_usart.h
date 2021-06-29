/*
 * bootloader_usart.h
 *
 *  Created on: Jun 29, 2021
 *      Author: ziadyasser
 */

#ifndef BOOTLOADER_INC_BOOTLOADER_USART_H_
#define BOOTLOADER_INC_BOOTLOADER_USART_H_

#include "stm32f4xx_usart.h"
#include "bootloader_cfg.h"
#include "bootloader_dma.h"



void BootLoader_USARTInit(uint32_t BootLoader_USARTBaudRate);
void BootLoader_USARTEnable(void);


#endif /* BOOTLOADER_INC_BOOTLOADER_USART_H_ */
