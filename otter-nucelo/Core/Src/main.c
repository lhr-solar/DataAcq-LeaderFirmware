/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f4xx_hal.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_UART5_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_UART5_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
  HAL_Delay(1000);
  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
  // HAL_Delay(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  char buffer[] = { 'H', 'e', 'l', 'l', 'o', '\0' }; // Manual



  CAN_FilterTypeDef filterConfig;


  // Set the bits of the 32bit ID Register
    // Note: does not matter what these are since Mask will be 0x00000000
  filterConfig.FilterIdHigh = 0x0000; 
  filterConfig.FilterIdLow = 0x0000;

  // Set the bits of the MASK Register. If a bit is set, the corresponding bit of the 
    // received ID will be compared with the corresponding bit of the ID Register. If 
    // all bits that the mask requies to be checked match, then the CAN ID is accepted. 
  filterConfig.FilterMaskIdHigh = 0x0000; 
  filterConfig.FilterMaskIdLow = 0x0000;
    // B/c we want to receive all CAN IDs, do not check any bits

  filterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;

  // This struct if for filter bank 0 
    // Note: can call the config filter with this struct again but change which 
    // bank we are talking about. 
  filterConfig.FilterBank = 0;

  // 2 Options: MASK mode or LIST mode 
    // Mask mode: use the Mask to compare the bits
    // List mode: you can only compare and accept two CAN IDs (if 32 bit ID register) or four CAN IDs (if two 16 bit regsiters)
  filterConfig.FilterMode = CAN_FILTERMODE_IDMASK;

  // 32 bit ID vs two 16 bit ID register
    // For us does not matter b/c its just all 0s 
  filterConfig.FilterScale = CAN_FILTERSCALE_32BIT;

  // Enable this filter bank
  filterConfig.FilterActivation = ENABLE;

  // We are not giving any filters to the slave can (CAN2)
    // We are only using CAN1 and CAN3 
  filterConfig.SlaveStartFilterBank = 0;

  HAL_CAN_ConfigFilter(&hcan1, &filterConfig);

  // temp
  HAL_CAN_Start(&hcan1);


  CAN_TxHeaderTypeDef   TxHeader;
  uint32_t              TxMailbox;  // while mailbox gets used is written here
  uint8_t               TxData[8];

  TxHeader.StdId = 0x11;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.DLC = 2;
  TxHeader.TransmitGlobalTime = DISABLE;

  TxData[0] = 0xCA;
  TxData[1] = 0xFE;
 
  
  // while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 3) {}

  while(1){
    HAL_Delay(100);
    /* Request transmission */

    // Wait for HAL Okay
    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
    // if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
    // {
    //   while(1){
    //     HAL_Delay(100);
    //     HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    //   }
    // }

    // Wait for the transmit to actuall happen
    // while (1) {
    //   uint32_t tsr = READ_REG(hcan1.Instance->TSR);
    //   if (tsr & (CAN_TSR_TXOK0 | CAN_TSR_TXOK1 | CAN_TSR_TXOK2)) break;
    //   HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    // }

    // Message Sent, toggle LED real quick
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    HAL_Delay(200);
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

    // while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 3) {
    //     HAL_Delay(2000);
    //     HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    // }

    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];

    // Check if there are messages in the FIFO
    if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0))
    {
        // Retrieve the received message
        if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK)
        {
            // Check if the message ID is 0x0FB
            // if (rxHeader.StdId == 0x0FB)
            // {
                // Handle received data (e.g., print, process, etc.)
            for(int i = 0; i < 100; i++){
              HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
              HAL_Delay(10);
            }
            // }
        }
    }


  }




  while (1){

    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];

    // Check if there are messages in the FIFO
    if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0))
    {
        // Retrieve the received message
        if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK)
        {
            // Check if the message ID is 0x0FB
            // if (rxHeader.StdId == 0x0FB)
            // {
                // Handle received data (e.g., print, process, etc.)
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
            // }
        }
    }

  }


  while (1)
  {
    HAL_Delay(1);



  // (+) HAL_CAN_AddTxMessage             : Add a message to the Tx mailboxes
  //     and activate the corresponding
  //     transmission request
  // (+) HAL_CAN_AbortTxRequest           : Abort transmission request
  // (+) HAL_CAN_GetTxMailboxesFreeLevel  : Return Tx mailboxes free level
  // (+) HAL_CAN_IsTxMessagePending       : Check if a transmission request is
  //     pending on the selected Tx mailbox
  // (+) HAL_CAN_GetRxMessage             : Get a CAN frame from the Rx FIFO

    // HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *hcan)



    // HAL_CAN_AddTxMessage(&hcan1, );
    // HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *hcan, const CAN_TxHeaderTypeDef *pHeader, 
    //                                         const uint8_t aData[], uint32_t *pTxMailbox)


    // HAL_CAN_GetRXMessage(&hcan1, CAN_RX_FIFO0, 
    // HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *hcan, uint32_t RxFifo, 
                                            // CAN_RxHeaderTypeDef *pHeader, uint8_t aData[])

    // HAL_CAN_Transmit()
    HAL_UART_Transmit(&huart2, (uint8_t*)(&(buffer[4])), 1, 1000);
    HAL_UART_Transmit(&huart2, (uint8_t*)(&(buffer[3])), 1, 1000);
    HAL_UART_Transmit(&huart2, (uint8_t*)(&(buffer[2])), 1, 1000);
    HAL_UART_Transmit(&huart2, (uint8_t*)(&(buffer[1])), 1, 1000);
    HAL_UART_Transmit(&huart2, (uint8_t*)(&(buffer[0])), 1, 1000);
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  // hcan1.Init.Mode = CAN_MODE_LOOPBACK;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 16;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC1 PC2 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA8 PA9 PA10
                           PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
