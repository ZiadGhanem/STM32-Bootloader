/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "crc.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BL_ACK 0x79
#define BL_NACK 0x1F
#define BL_Version 0x1A
#define BL_Num_Commands 11UL
#define BL_Receive_Buffer_Size	255UL
#define BL_Transmit_Buffer_Size	255UL
/* Bootloader supported commands */
#define BL_Get 0x00
#define BL_Get_Version_And_Protection 0x01
#define	BL_Get_ID	0x02
#define	BL_Read_Memory	0x11
#define	BL_Go	0x21
#define	BL_Write_Memory	0x31
#define	BL_Erase	0x43
#define	BL_Write_Protect	0x62
#define	BL_Write_Unprotect	0x73
#define	BL_Readout_Protect	0x82
#define	BL_Readout_Unprotect	0x92
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Supported commands */
static const uint8_t BL_Commands[] =
	{0x00, 0x01, 0x02, 0x11, 0x21, 0x31, 0x43, 0x62, 0x73, 0x82, 0x92};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static ErrorStatus BL_VerifyCommand(uint8_t Command, uint8_t CommandComplement);
static ErrorStatus BL_VerifyChecksum(uint8_t* pData, uint32_t Length);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
ErrorStatus BL_VerifyCommand(uint8_t Command, uint8_t CommandComplement)
{
	ErrorStatus CommandExists = ERROR;
	uint8_t i;
	/* Verify that the command exists */
	for(i = 0; i < BL_Num_Commands; i++)
	{
		if(Command == BL_Commands[i])
		{
			CommandExists = SUCCESS;
			break;
		}
		else
		{

		}
	}
	if((CommandExists == SUCCESS) && ((Command ^ CommandComplement) == 0xFF))
	{
		return SUCCESS;
	}
	else
	{
		return ERROR;
	}
}
static ErrorStatus BL_VerifyChecksum(uint8_t* pData, uint32_t Length)
{
	uint8_t result = 0x00;
	for(int i = 0; i < Length; i++)
	{
		result ^= pData[i];
	}
	if(result == 0x00)
	{
		return SUCCESS;
	}
	else
	{
		return ERROR;
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	/* Bootloader Receive Buffer */
	static uint8_t BL_ReceiveBuffer[BL_Receive_Buffer_Size];
	/* Bootloader Transmit Buffer */
	static uint8_t BL_TransmitBuffer[BL_Transmit_Buffer_Size];
	/* Bootloader current Systick */
	uint32_t BL_CurrentTick = 0;
	/* Used for storing read or write addresses */
	uint8_t* BL_Address;
	/* Used for storing number of read or written bytes */
	uint8_t BL_NumBytes;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CRC_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	/* Start bootloader */
	if(GPIO_PIN_SET ==  HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin))
	{
		while(1)
		{
			/* Flash green LED */
			if(HAL_GetTick() - BL_CurrentTick > 500)
			{
				HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
				BL_CurrentTick = HAL_GetTick();
			}
			/* Wait for command */
			if(HAL_OK == HAL_UART_Receive(&huart1, BL_ReceiveBuffer, 2, 5))
			{
				/* Verify that it is a valid command */
				if(SUCCESS == BL_VerifyCommand(BL_ReceiveBuffer[0], BL_ReceiveBuffer[1]))
				{
					switch(BL_ReceiveBuffer[0])
					{
						case BL_Get:
							/* Send ACK byte */
							BL_TransmitBuffer[0] = BL_ACK;
							/* Send the number of bytes (version + commands) */
							BL_TransmitBuffer[1] = BL_Num_Commands;
							/* Send the bootloader version */
							BL_TransmitBuffer[2] = BL_Version;
							HAL_UART_Transmit(&huart1, BL_TransmitBuffer, 3, HAL_MAX_DELAY);
							/* Send the supported commands */
							HAL_UART_Transmit(&huart1, (uint8_t*)BL_Commands, BL_Num_Commands, HAL_MAX_DELAY);
							/* Send ACK byte */
							BL_TransmitBuffer[0] = BL_ACK;
							HAL_UART_Transmit(&huart1, BL_TransmitBuffer, 1, HAL_MAX_DELAY);
							break;
						case BL_Get_Version_And_Protection:
							/* Send ACK byte */
							BL_TransmitBuffer[0] = BL_ACK;
							/* Send the bootloader version */
							BL_TransmitBuffer[1] = BL_Version;
							/* Send option bytes */
							BL_TransmitBuffer[2] = 0x00;
							BL_TransmitBuffer[3] = 0x00;
							/* Send ACK byte */
							BL_TransmitBuffer[4] = BL_ACK;
							HAL_UART_Transmit(&huart1, BL_TransmitBuffer, 5, HAL_MAX_DELAY);
							break;
						case BL_Get_ID:
							/* Send ACK byte */
							BL_TransmitBuffer[0] = BL_ACK;
							/* Send the number of bytes */
							BL_TransmitBuffer[1] = 1;
							/* Send the process ID */
							BL_TransmitBuffer[2] = 0x04;
							BL_TransmitBuffer[3] = 0x00;
							/* Send ACK byte */
							BL_TransmitBuffer[4] = BL_ACK;
							HAL_UART_Transmit(&huart1, BL_TransmitBuffer, 5, HAL_MAX_DELAY);
							break;
						case BL_Read_Memory:
							/* Is RDP active ? */
							if((*(__IO uint8_t*)(OPTCR_BYTE1_ADDRESS) == (uint8_t)OB_RDP_LEVEL_0))
							{
								/* Send ACK byte */
								BL_TransmitBuffer[0] = BL_ACK;
								HAL_UART_Transmit(&huart1, BL_TransmitBuffer, 1, HAL_MAX_DELAY);
								/* Receive the start address (4 bytes) with checksum */
								if(HAL_OK == HAL_UART_Receive(&huart1, BL_ReceiveBuffer, 5, HAL_MAX_DELAY))
								{
									/* Address valid and checksum ok ? */
									if(SUCCESS == BL_VerifyChecksum(BL_ReceiveBuffer, 5))
									{
										BL_Address = (uint8_t*)(*(uint32_t*)BL_ReceiveBuffer);
										/* Send ACK byte */
										BL_TransmitBuffer[0] = BL_ACK;
										HAL_UART_Transmit(&huart1, BL_TransmitBuffer, 1, HAL_MAX_DELAY);
										/* Receive the number of bytes to be read and a checksum */
										if(HAL_OK == HAL_UART_Receive(&huart1, BL_ReceiveBuffer, 2, HAL_MAX_DELAY))
										{
											/* Checksum ok ? */
											if(SUCCESS == BL_VerifyChecksum(BL_ReceiveBuffer, 2))
											{
												BL_NumBytes = BL_ReceiveBuffer[0];
												/* Send ACK byte */
												BL_TransmitBuffer[0] = BL_ACK;
												HAL_UART_Transmit(&huart1, BL_TransmitBuffer, 1, HAL_MAX_DELAY);
												/* Send data to host */
												HAL_UART_Transmit(&huart1, BL_Address, BL_NumBytes, HAL_MAX_DELAY);
											}
											else
											{
												/* Send NACK byte */
												BL_TransmitBuffer[0] = BL_NACK;
												HAL_UART_Transmit(&huart1, BL_TransmitBuffer, 1, HAL_MAX_DELAY);
											}
										}
										else
										{
											break;
										}
									}
									else
									{
										/* Send NACK byte */
										BL_TransmitBuffer[0] = BL_NACK;
										HAL_UART_Transmit(&huart1, BL_TransmitBuffer, 1, HAL_MAX_DELAY);
									}
								}
								else
								{
									break;
								}
							}
							else
							{
								/* Send NACK byte */
								BL_TransmitBuffer[0] = BL_NACK;
								HAL_UART_Transmit(&huart1, BL_TransmitBuffer, 1, HAL_MAX_DELAY);
							}
							break;
						default:
							break;
					}
				}
				else
				{
					/* Send NACK byte */
					BL_TransmitBuffer[0] = BL_NACK;
					HAL_UART_Transmit(&huart1, BL_TransmitBuffer, 1, HAL_MAX_DELAY);
				}


			}
			else
			{

			}
		}
	}
	/* Start application */
	else
	{
		while(1);
	}
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
