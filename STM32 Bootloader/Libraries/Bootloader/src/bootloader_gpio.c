/*
 * bootloader_gpio.c
 *
 *  Created on: Jun 29, 2021
 *      Author: ziadyasser
 */

#include "bootloader_gpio.h"

void BootLoader_GPIOInit(void)
{
	/* create the GPIO Initialization structure */
	GPIO_InitTypeDef GPIO_InitStruct;
	/* Enable GPIO Clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	/* Set UART pins alternate function */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
	/* Initialize the GPIO initialization structure */
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	/* Initialize the GPIO */
	GPIO_Init(GPIOA, &GPIO_InitStruct);
}
