/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../../def.h"
#include <stdatomic.h>
//#include <stdio.h>
#include "../../SEGGER/SEGGER_RTT.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_tx;
DMA_HandleTypeDef hdma_i2c1_rx;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

DMA_HandleTypeDef hdma_memtomem_dma1_channel1;
/* USER CODE BEGIN PV */
//DMA濠电偞鍨堕幐鏄忋亹閸愵厹浜规繛鎴欏灩缁犳娊鏌曟径娑㈡闁哄棭浜炵槐鎾诲磼濞戞瑥纰嶉梺鍝ュ枎濞�??崇暦閻橀潧濮??
uint8_t dmaRxBufferA[BUFFER_SIZE];
volatile uint8_t dmaRxBufferALock = 0;
//DMA濠电偞鍨堕幐鏄忋亹閸愵厹浜规繛鎴欏灩缁犳娊鏌曟径娑㈡闁哄棭浜炵槐鎾诲磼濞戞瑥纰嶉梺鍝ュ枎濞�??崇暦閻橀潧濮??
uint8_t dmaRxBufferB[BUFFER_SIZE];

//DMA 濠电偞鍨堕幐鏄忋亹閸愵厹浜规繛鎴欏灩閻鏌????婵炶揪缍?濞咃絾寰勯崟顖涚厱闁归偊鍘肩徊濠氭�????
uint8_t dmaTxBuffer[BUFFER_SIZE];
spinlock_t dmaTxBufferLock = { 0};

/// 濠电姭鎷冮崨顓濇闂佸搫妫濇禍璺侯嚕椤愶絽顕遍柡澶嬪灦椤??
volatile uint8_t dmaRxBufferLen = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void buffaFreeA(DMA_HandleTypeDef *hdma)
{
	HAL_GPIO_TogglePin(StatLED_GPIO_Port, StatLED_Pin);
	
	spin_unlock(&gInCmdBufferLock);
	atomic_fetch_add(&gInCmdBufferIdx, dmaRxBufferLen);
	
}


void errorFreeA(DMA_HandleTypeDef *hdma)
{
	spin_unlock(&gInCmdBufferLock);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
	{
		//SEGGER_RTT_printf(0, "RX:\n");
		//dump(ptr, Size);
		spin_unlock(&dmaTxBufferLock);
	}
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
	{
		spin_unlock(&dmaTxBufferLock);
	}
	
}


void dump(void* addr, int len) {
	unsigned char* p = (unsigned char*) addr;
	for (int i = 0; i < len; i += 16) {
		SEGGER_RTT_printf(0,"%08x: ", (unsigned int) p + i);
		for (int j = 0; j < 16; j++) {
			if (i + j < len) {
				SEGGER_RTT_printf(0,"%02x ", p[i + j]);
			}
			else {
				SEGGER_RTT_printf(0,"   ");
			}
			if (j == 7) {
				SEGGER_RTT_printf(0," ");
			}
		}
		SEGGER_RTT_printf(0," ");
		for (int j = 0; j < 16; j++) {
			if (i + j < len) {
				unsigned char c = p[i + j];
				if (c < 32 || c >= 127) {
					c = '.';
				}
				SEGGER_RTT_printf(0,"%c", c);
			}
		}
		SEGGER_RTT_printf(0,"\n");
	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	/* Prevent unused argument(s) compilation warning */
	/* NOTE : This function should not be modified, when the callback is needed,
	          the HAL_UARTEx_RxEventCallback can be implemented in the user file.
	       */
	//uint8_t* ptr = NULL;

	if (huart->Instance == USART1)
	{
		//aRxBufferIdx = Size;
		//HAL_DMA_Start_IT(&hdma_memtomem_dma1_channel1,)
		//SyncReceivedDataToCmdBuffer1(huart->pRxBuffPtr, Size);
		//atomic_store(&m2_SystemTimeout, HAL_GetTick());
	
		if (huart->RxState ==  HAL_UART_STATE_READY)
		{			
			/// 缂傚倷鐒︾粙鎴λ囬婊勵偨闁绘梻鍘ч惌妤併亜閺嶃劎鐭嬮柡??
			while (HAL_DMA_GetState(&hdma_memtomem_dma1_channel1) != HAL_DMA_STATE_READY)
			{
				
			}
			hdma_memtomem_dma1_channel1.XferCpltCallback = &buffaFreeA;
			hdma_memtomem_dma1_channel1.XferErrorCallback = &errorFreeA;
			dmaRxBufferLen = Size;
			uint8_t* ptr = huart->pRxBuffPtr;
			
			if (huart->pRxBuffPtr == dmaRxBufferA)
			{
				if (HAL_UARTEx_ReceiveToIdle_DMA(huart, dmaRxBufferB, sizeof(dmaRxBufferB)) != HAL_OK)
				{
					Error_Handler();
				}
			}
			else
			{
				if (HAL_UARTEx_ReceiveToIdle_DMA(huart, dmaRxBufferA, sizeof(dmaRxBufferA)) != HAL_OK)
				{
					Error_Handler();
				}
			}
			//
			//SEGGER_RTT_printf(0, "RX:\n");
			//dump(ptr, Size);
			
			
			uint32_t idx = __LDRBT(&gInCmdBufferIdx);
			uint8_t* dstPtr = &mccEvm.gInCmdBuffer[idx];
			
			spin_lock(&gInCmdBufferLock);
			HAL_DMA_Start_IT(&hdma_memtomem_dma1_channel1, (uint32_t)ptr, (uint32_t)dstPtr, Size);
			
		}
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

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	
	InitBuffer();
	
	

	//__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	
	//__HAL_LINKDMA(&huart1, hdmarx, hdma_usart1_rx);
	
	//HAL_DMA_RegisterCallback(&hdma_usart1_rx,)
	
	
	
	//HAL_UART_Receive_DMA(&huart1, gInCmdBuffer, sizeof(gInCmdBuffer));
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, dmaRxBufferA,sizeof(dmaRxBufferA));
	
	//__HAL_DMA_ENABLE_IT()
	
	//HAL_UARTEx_ReceiveToIdle_IT(&huart1, gInCmdBuffer, sizeof(aRxBuffer));
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //SyncReceivedDataToCmdBuffer();

	  ProcessCommand();
	  
	  //HAL_GetTick();
	  
	  
	  /// 闂備礁鎲￠悷锕傚�???婵炶揪绲块弻澶愩?呴锔界厱婵炲棙鐟ч幃鑲╃磼閹插鐣电?规洘锕㈤獮鎾诲箳閹达附顔冮梻浣圭湽閸斿瞼?姘煎弮�?�娊鎮㈤悡搴ｎ�??
	  if (aTxBufferIdx > 0)
	  {
	  
		  spin_lock(&dmaTxBufferLock);
		  uint32_t idx = aTxBufferIdx;
		  memcpy(dmaTxBuffer, aTxBuffer, idx);
		  __STRBT(0, &aTxBufferIdx);
		  
		  //SEGGER_RTT_printf(0, "TX:\n");
		  //dump(dmaTxBuffer, idx);
		  
		  HAL_UART_Transmit_DMA(&huart1, dmaTxBuffer, idx);
		  
		  //ITM_Writes("ABCD", 4);
		  
		  //printf("DCBA");
		  //SEGGER_RTT_printf(0,"abcdef");
		  
	  }
	  
	  if (mccEvm.cmd_received == 1)
	  {
		  mccEvm.cmd_received = 0;
		  mccEvm.EndOfWrite = 1;
		  
		  MCC_Decode();
		  
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO, RCC_MCO1SOURCE_PLLCLK, RCC_MCODIV_1);
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  * Configure DMA for memory to memory transfers
  *   hdma_memtomem_dma1_channel1
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* Configure DMA request hdma_memtomem_dma1_channel1 on DMA1_Channel1 */
  hdma_memtomem_dma1_channel1.Instance = DMA1_Channel1;
  hdma_memtomem_dma1_channel1.Init.Direction = DMA_MEMORY_TO_MEMORY;
  hdma_memtomem_dma1_channel1.Init.PeriphInc = DMA_PINC_ENABLE;
  hdma_memtomem_dma1_channel1.Init.MemInc = DMA_MINC_ENABLE;
  hdma_memtomem_dma1_channel1.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_memtomem_dma1_channel1.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_memtomem_dma1_channel1.Init.Mode = DMA_NORMAL;
  hdma_memtomem_dma1_channel1.Init.Priority = DMA_PRIORITY_LOW;
  if (HAL_DMA_Init(&hdma_memtomem_dma1_channel1) != HAL_OK)
  {
    Error_Handler( );
  }

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(StatLED_GPIO_Port, StatLED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : StatLED_Pin */
  GPIO_InitStruct.Pin = StatLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(StatLED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
