/**
	******************************************************************************
  * @file    LibJPEG/LibJPEG_Decoding/Src/main.c
  * @author  MCD Application Team
  * @brief   Main program body
  *          This sample code shows how to decompress JPEG file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
FATFS SDFatFs;  /* File system object for SD card logical drive */
FIL MyFile;     /* File object */
char SDPath[4]; /* SD card logical drive path */

RGB_typedef *RGB_matrix;
uint8_t _aucLine[2048];
uint32_t line_counter = 0;
uint16_t Xpos = 0;
uint16_t Ypos = 0;
uint32_t byteswritten;
/* UART handler declaration */
UART_HandleTypeDef UartHandle;

/* Buffer used for reception */
uint8_t pkgBufList[10][10000];
uint8_t pkgBufList2[10][10000];
uint8_t disPlayBuf[40];
uint8_t tmpBuf[30];

FRESULT res;
__IO ITStatus UartReady = RESET;
__IO uint32_t UserButtonStatus = 0;
int colPtr = 0, rowPtr = 0, BufChoose = 1;
/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static uint8_t Jpeg_CallbackFunction(uint8_t* Row, uint32_t DataLength);
static void LCD_Config(void);
static void Error_Handler(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
   /* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch, instruction and Data caches
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Global MSP (MCU Support Package) initialization
     */
  HAL_Init();

  /* Configure the system clock to 180 MHz */
  SystemClock_Config();

  /* Configure LED3 */
  BSP_LED_Init(LED3);
	BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);
  /*##-1- LCD Configuration ##################################################*/
  LCD_Config();
	
	UartHandle.Instance        = USARTx;

  UartHandle.Init.BaudRate   = 57600;
  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits   = UART_STOPBITS_1;
  UartHandle.Init.Parity     = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode       = UART_MODE_TX_RX;
  if(HAL_UART_DeInit(&UartHandle) != HAL_OK)
  {
    Error_Handler();
  }  
  if(HAL_UART_Init(&UartHandle) != HAL_OK)
  {
    Error_Handler();
  }
  if(FATFS_LinkDriver(&SD_Driver, SDPath) == 0)
	{
		if(f_mount(&SDFatFs, (TCHAR const*)SDPath, 0) == FR_OK)
		{
			while (1)
			{
				HAL_UART_Receive_IT(&UartHandle, (uint8_t *)tmpBuf, 1);
				sprintf((char *)disPlayBuf,"  [buf=%d col=%d row=%d]%dbps",BufChoose,colPtr,rowPtr,UartHandle.Init.BaudRate);
				BSP_LCD_DisplayStringAt(0, 0, (uint8_t *)disPlayBuf, CENTER_MODE);
				if(UserButtonStatus == 1)
				{
					if(f_open(&MyFile, "image2.jpg", FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) 
						{
							Error_Handler();
						}
						else
						{
							res = f_write(&MyFile, pkgBufList, sizeof(pkgBufList), (void *)&byteswritten);
							
							if((byteswritten == 0) || (res != FR_OK))
							{
								Error_Handler();
							}
							else
							{
								f_close(&MyFile);
								if(f_open(&MyFile, "image2.jpg", FA_OPEN_APPEND | FA_WRITE) != FR_OK) 
								{
									Error_Handler();
								}
								else
								{
									res = f_write(&MyFile, pkgBufList2, sizeof(pkgBufList2), (void *)&byteswritten);
									if((byteswritten == 0) || (res != FR_OK))
									{
										Error_Handler();
									}
									else
									{
										f_close(&MyFile);
									}
								}
								if(f_open(&MyFile, "image2.jpg", FA_READ) != FR_OK)
								{
									Error_Handler();
								}
								else
								{
									jpeg_decode(&MyFile, IMAGE_HEIGHT, _aucLine, Jpeg_CallbackFunction);
									BSP_LCD_DisplayStringAt(0, 0, (uint8_t *)"			a			", CENTER_MODE);
									f_close(&MyFile);
									UserButtonStatus = 0;
									BufChoose = 1;
									rowPtr = 0;
									colPtr = 0;
									line_counter = 0;
									for(int count = 0; count < 10; count ++)
									{
										for(int count2 = 0; count2 < 10000; count2 ++)
										{
											pkgBufList[count][count2] = '\0';
										}
									}
									for(int count = 0; count < 10; count ++)
									{
										for(int count2 = 0; count2 < 10000; count2 ++)
										{
											pkgBufList2[count][count2] = '\0';
										}
									}
								}
							}
						}
					}
				}
			}
		}
		else
		{
			Error_Handler();
		}
}

/**
  * @brief  LCD Configuration.
  * @retval None
  */

static void LCD_Config(void)
{
  uint8_t lcd_status = LCD_OK;

  /* LCD DSI initialization */
  lcd_status = BSP_LCD_Init();
  if(lcd_status != LCD_OK)
  {
    Error_Handler();
  }
  
  BSP_LCD_LayerDefaultInit(0, LCD_FB_START_ADDRESS);
  BSP_LCD_SelectLayer(0);  

  /* Clear the LCD Background layer */
  BSP_LCD_Clear(LCD_COLOR_WHITE);

  BSP_LCD_SetFont(&LCD_DEFAULT_FONT);

  /* Clear the LCD */
  BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
  BSP_LCD_Clear(LCD_COLOR_WHITE);

   Xpos = (uint16_t)((BSP_LCD_GetXSize() - IMAGE_WIDTH) / 2);
   Ypos = (uint16_t)((BSP_LCD_GetYSize() - IMAGE_HEIGHT) / 2);
}

/**
  * @brief  Copy decompressed data to display buffer.
  * @param  Row: Output row buffer
  * @param  DataLength: Row width in output buffer
  * @retval None
  */
static uint8_t Jpeg_CallbackFunction(uint8_t* Row, uint32_t DataLength)
{
  uint32_t i = 0;
  RGB_matrix =  (RGB_typedef*)Row;
  uint32_t  ARGB8888Buffer[IMAGE_WIDTH];

#ifdef SWAP_RB
  for(i = 0; i < IMAGE_WIDTH; i++)
  {
    ARGB8888Buffer[i]  = (uint32_t)
      (
        0xFF000000                                       |
       (((uint32_t)(RGB_matrix[i].B) & 0x000000FF) >> 0) |
       (((uint32_t)(RGB_matrix[i].G) & 0x000000FF) << 8) |
       (((uint32_t)(RGB_matrix[i].R) & 0x000000FF) << 16)
      );

    BSP_LCD_DrawPixel((i + Xpos), (line_counter + Ypos), ARGB8888Buffer[i]);
  }
#else
	uint8_t wd = DataLength;
  for(i = 0; i < IMAGE_WIDTH; i++)
  {
    ARGB8888Buffer[i]  = (uint32_t)
      (
        0xFF000000                                       |
       (((uint32_t)(RGB_matrix[i].R) & 0x000000FF) >> 0) |
       (((uint32_t)(RGB_matrix[i].G) & 0x000000FF) << 8) |
       (((uint32_t)(RGB_matrix[i].B) & 0x000000FF) << 16)
      );

    BSP_LCD_DrawPixel((i + Xpos), (line_counter + Ypos), ARGB8888Buffer[i]);
//		BSP_LCD_DrawPixel((i + Xpos), (line_counter + Ypos), ARGB8888Buffer[i]);
  }
#endif
  line_counter++;
  return 0;
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 180000000
  *            HCLK(Hz)                       = 180000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 360
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            PLL_R                          = 6
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  HAL_StatusTypeDef ret = HAL_OK;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the device is
     clocked below the maximum system frequency, to update the voltage scaling value
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
#if defined(USE_STM32469I_DISCO_REVA)
  RCC_OscInitStruct.PLL.PLLM = 25;
#else
  RCC_OscInitStruct.PLL.PLLM = 8;
#endif /* USE_STM32469I_DISCO_REVA */
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 6;

  ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }

  /* Activate the OverDrive to reach the 180 MHz Frequency */
  ret = HAL_PWREx_EnableOverDrive();
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	switch(BufChoose)
	{
		case 1:
			pkgBufList[colPtr][rowPtr] = tmpBuf[0];
			break;
		case 2:
			pkgBufList2[colPtr][rowPtr] = tmpBuf[0];
			break;
	}
	if(tmpBuf[0] == '!')
	switch(BufChoose)
	{
		case 1:
			if(rowPtr == 0)
			{
				if(pkgBufList[colPtr - 1][9999] == '!')
				UserButtonStatus = 1;
			}
			else if(pkgBufList[colPtr][rowPtr - 1] == '!')
			{
				UserButtonStatus = 1;
			}
			break;
		case 2:
			if(rowPtr == 0)
			{
				if(pkgBufList2[colPtr - 1][9999] == '!')
				UserButtonStatus = 1;
			}
			else if(pkgBufList2[colPtr][rowPtr - 1] == '!')
			{
				UserButtonStatus = 1;
			}
			break;
		default:
			break;
	}
	rowPtr++;
	if(rowPtr > 9999)
	{
		colPtr++;
		rowPtr = 0;
	}
	else if(colPtr > 9)
	{
		BufChoose++;
		colPtr = 0;
	}
	HAL_UART_Receive_IT(UartHandle, (uint8_t *)tmpBuf, 1);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
  /* Turn LED4 on: Transfer error in reception/transmission process */
  BSP_LED_On(LED4); 
}

/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(UserButtonStatus == 0)
		UserButtonStatus = 1;
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
    /* Turn LED3 on */
    BSP_LED_On(LED3);
    while(1)
    {
    }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
