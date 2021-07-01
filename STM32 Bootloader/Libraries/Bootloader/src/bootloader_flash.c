/*
 * bootloader_flash.c
 *
 *  Created on: Jun 29, 2021
 *      Author: ziadyasser
 */

#include "bootloader_flash.h"

uint8_t BootLoader_FlashGetReadoutProtectionStatus(void)
{
	return *(__IO uint8_t*)(OPTCR_BYTE1_ADDRESS);
}
