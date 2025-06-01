/* USER CODE BEGIN Header */
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
#include "stm32f4xx_hal_can.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_uart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct __attribute__((__packed__)) {
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];
} CAN_UART_Packet;

typedef struct __attribute__((__packed__)) {
    uint16_t can_id;      // 0 to 0x7FF
    uint16_t time_stamp;  // 10ms period (tim2)
    uint8_t  num_bytes;   // 0 to 8 bytes of data
    uint8_t  data[8];     // data payload
} CAN_FORMATTED_Packet;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// For UART2 Reception 
// -----------------------------
volatile uint8_t rx_buffer;
#define FIFO_SIZE 64  
volatile uint8_t fifo[FIFO_SIZE];
volatile uint16_t fifo_head = 0;
volatile uint16_t fifo_tail = 0;
// -----------------------------

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_UART5_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */
void read_meta_data(void);
void send_AT_cmd(char* tx_msg, uint32_t tx_length, char* rx_msg, uint32_t rx_length, uint32_t delay);
void format_UART_Msg(char* msg, CAN_FORMATTED_Packet* formatted_msg);
void parse_RX_CAN(CAN_UART_Packet* rx_msg, CAN_FORMATTED_Packet* formatted_msg, uint16_t time_stamp);
void transmit_ASCII_CAN_Packet(CAN_FORMATTED_Packet* formatted_msg);
int  UART_FIFO_get_Char(char* rx_char);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef struct{
    GPIO_TypeDef *GPIOx;
    uint16_t GPIO_PIN;

} gpio_t;

static const gpio_t ss_l[] = {
    {GPIOB, GPIO_PIN_10}, 
    {GPIOB, GPIO_PIN_1}, 
    {GPIOB, GPIO_PIN_14}, 
    {GPIOB, GPIO_PIN_2}, 
    {GPIOB, GPIO_PIN_0}, 
    {GPIOC, GPIO_PIN_5}, 
    {GPIOC, GPIO_PIN_4}
};

static const gpio_t ss_r[] = {
    {GPIOC, GPIO_PIN_6}, 
    {GPIOC, GPIO_PIN_7}, 
    {GPIOB, GPIO_PIN_15}, 
    {GPIOA, GPIO_PIN_15}, 
    {GPIOC, GPIO_PIN_10}, 
    {GPIOC, GPIO_PIN_11}, 
    {GPIOC, GPIO_PIN_12}
};

void iter(void){
    static int i = 0;
    HAL_GPIO_TogglePin(ss_r[i].GPIOx, ss_r[i].GPIO_PIN);
    HAL_GPIO_TogglePin(ss_l[i].GPIOx, ss_l[i].GPIO_PIN);
    i = (i + 1) % 7;
}

static const gpio_t arr[] = {
        {GPIOB, GPIO_PIN_10 },
        {GPIOC, GPIO_PIN_6  },
        {GPIOC, GPIO_PIN_7  },
        {GPIOC, GPIO_PIN_12 },
        {GPIOC, GPIO_PIN_4  },
        {GPIOB, GPIO_PIN_0  },
        {GPIOB, GPIO_PIN_2  },
        {GPIOA, GPIO_PIN_15 },
        {GPIOB, GPIO_PIN_15 },
        {GPIOC, GPIO_PIN_12 },
        {GPIOC, GPIO_PIN_4  }
        };

void cyc(void){
    static int i = 1;
    HAL_GPIO_WritePin(arr[i].GPIOx, arr[i].GPIO_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(arr[(i+1)%11].GPIOx, arr[(i+1)%11].GPIO_PIN, GPIO_PIN_RESET);
    i = (i + 1) % 11;
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
  MX_CAN1_Init();
  MX_CAN3_Init();
  MX_USART2_UART_Init();
  MX_UART5_Init();
  MX_TIM14_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  CAN_FilterTypeDef filterConfig;

  filterConfig.FilterIdHigh = 0x0000;
  filterConfig.FilterIdLow  = 0x0000;

  filterConfig.FilterMaskIdHigh = 0x0000;
  filterConfig.FilterMaskIdLow  = 0x0000;

  filterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  filterConfig.FilterBank = 0;
  filterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  filterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  filterConfig.FilterActivation = ENABLE;
  filterConfig.SlaveStartFilterBank = 0;

  HAL_CAN_ConfigFilter(&hcan1, &filterConfig);

  HAL_CAN_Start(&hcan1);
  HAL_TIM_Base_Start(&htim14);
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_Base_Start(&htim5);

  // NOTE: ############ Very important for UART2 ISR reception ################
  __enable_irq();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  uint8_t sof[] = "SOF:  \n\r"; // 8 bytes to fit can formated packet dimensions
  CAN_FORMATTED_Packet sof_msg;

  uint8_t count = 0;
  char uartBuff[32];
  int uartBuffLen = 0;

  CAN_UART_Packet cu_packet;

  // Supposedly needed for UART 2 Rx
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);


  // HAL_Delay(10000);
  while (1)
  {

      // Transmit Start of Frame:
      format_UART_Msg(sof, &sof_msg); // Re-format every time, to get updated time stamp
      HAL_UART_Transmit(&huart2, (uint8_t*)&sof_msg, sizeof(sof_msg), HAL_MAX_DELAY);
      transmit_ASCII_CAN_Packet(&sof_msg);

      // Transmit Iteration Count: 
      count = (count + 1) % 255;
      uartBuffLen = sprintf(uartBuff, "Iteration: %u \n\r", count);
      HAL_UART_Transmit(&huart2, (uint8_t *)uartBuff, uartBuffLen, HAL_MAX_DELAY);

      cyc();
      read_meta_data();

      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_3);
      HAL_Delay(500);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  hcan1.Init.Prescaler = 96;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  //hcan1.Init.Mode = CAN_MODE_LOOPBACK;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
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
  * @brief CAN3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN3_Init(void)
{

  /* USER CODE BEGIN CAN3_Init 0 */

  /* USER CODE END CAN3_Init 0 */

  /* USER CODE BEGIN CAN3_Init 1 */

  /* USER CODE END CAN3_Init 1 */
  hcan3.Instance = CAN3;
  hcan3.Init.Prescaler = 16;
  hcan3.Init.Mode = CAN_MODE_NORMAL;
  hcan3.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan3.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan3.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan3.Init.TimeTriggeredMode = DISABLE;
  hcan3.Init.AutoBusOff = DISABLE;
  hcan3.Init.AutoWakeUp = DISABLE;
  hcan3.Init.AutoRetransmission = DISABLE;
  hcan3.Init.ReceiveFifoLocked = DISABLE;
  hcan3.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN3_Init 2 */

  /* USER CODE END CAN3_Init 2 */

}

static void MX_TIM2_Init(void)
{
  __HAL_RCC_TIM2_CLK_ENABLE();
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 48000 - 1;     // Divides 48 MHz down to 1 kHz (1 ms ticks)
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFFFFFF;       // Max 32-bit
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK){Error_Handler();}
}

static void MX_TIM5_Init(void)
{
  __HAL_RCC_TIM5_CLK_ENABLE();
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 48000 - 1;    // 1 ms tick (48 MHz / 48000 = 1000 Hz)
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 0xFFFFFFFF;       // Max 32-bit
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
      Error_Handler();
  }
}


/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 48000 - 1;  // 10 ms per tick at 48 MHz clock
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 65535;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);

  /*Configure GPIO pins : PC3 PC4 PC5 PC6
                           PC7 PC10 PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10
                           PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        // Put RX byte into circular SW FIFO
        uint16_t next_head = (fifo_head + 1) % FIFO_SIZE;
        if (next_head != fifo_tail) {  // Check for overflow
            fifo[fifo_head] = rx_buffer;
            fifo_head = next_head;
        } else {
          cyc();
        }
        // Re-enable interrupt for next byte
        HAL_UART_Receive_IT(&huart2, &rx_buffer, 1);
    }
}

void USART2_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart2);
}

// Returns 1 = SUCCESS
// Returns 0 = FAIL
int UART_FIFO_get_Char(char* rx_char) {
    if (fifo_head == fifo_tail) {
        return 0;  // FIFO empty
    }

    // Non empty:
    *rx_char = fifo[fifo_tail];
    fifo_tail = (fifo_tail + 1) % FIFO_SIZE;
    return 1;
}

void transmit_ASCII_CAN_Packet(CAN_FORMATTED_Packet* msg){
      char uartBuff[32];
      int uartBuffLen = 0;
      uartBuffLen = sprintf(uartBuff, "-------------------------\n\r");
      HAL_UART_Transmit(&huart2, (uint8_t *)uartBuff, uartBuffLen, HAL_MAX_DELAY);
      uartBuffLen = sprintf(uartBuff, "CAN ID: 0x%X \n\r", msg->can_id);
      HAL_UART_Transmit(&huart2, (uint8_t *)uartBuff, uartBuffLen, HAL_MAX_DELAY);
      uartBuffLen = sprintf(uartBuff, "Time Stamp: %u \n\r", msg->time_stamp);
      HAL_UART_Transmit(&huart2, (uint8_t *)uartBuff, uartBuffLen, HAL_MAX_DELAY);
      uartBuffLen = sprintf(uartBuff, "Num Data Bytes: %u \n\r", msg->num_bytes);
      HAL_UART_Transmit(&huart2, (uint8_t *)uartBuff, uartBuffLen, HAL_MAX_DELAY);
      uartBuffLen = sprintf(uartBuff, "Data: %s \n\r", msg->data);
      HAL_UART_Transmit(&huart2, (uint8_t *)uartBuff, uartBuffLen, HAL_MAX_DELAY);
      uartBuffLen = sprintf(uartBuff, "-------------------------\n\r");
      HAL_UART_Transmit(&huart2, (uint8_t *)uartBuff, uartBuffLen, HAL_MAX_DELAY);
}


// Expects ASCII msg (max 8 bytes)
void format_UART_Msg(char* msg, CAN_FORMATTED_Packet* formatted_msg){

    // Data Acq CAN id
    formatted_msg->can_id     = 0x300;
      // Used by other systems: 1,2,4,5,6. Max = x7ff, so we will use 0x3XX

    // 10 ms time stamp
    volatile uint32_t timer_val = __HAL_TIM_GET_COUNTER(&htim2);
    uint16_t time_stamp = ((timer_val / 20)&0xFFFF); // 10ms
    formatted_msg->time_stamp = time_stamp;

    // Max of 8 bytes
    formatted_msg->num_bytes  = strlen((char*)msg);
    if(formatted_msg->num_bytes > 8){formatted_msg->num_bytes = 8;}

    // Copy the data package over
    for(int i = 0; i < 8; i ++){
      formatted_msg->data[i]  = (uint8_t)(msg[i]);
    }
}

// Turn raw recieved can into formated CAN
void parse_RX_CAN(CAN_UART_Packet* rx_msg, CAN_FORMATTED_Packet* formatted_msg, uint16_t time_stamp){

    formatted_msg->can_id     = rx_msg->header.StdId;
    formatted_msg->time_stamp = time_stamp;
    formatted_msg->num_bytes  = rx_msg->header.DLC;

    for(int i = 0; i < 8; i ++){
      formatted_msg->data[i]  = rx_msg->data[i];
    }
}

void read_meta_data(void){
    // XBee needs 1 second of silence before and after sending "+++"
    HAL_Delay(1001); 

    // Step 0: Arm reception of UART2 data
    HAL_UART_Receive_IT(&huart2, &rx_buffer, 1);  // Receive 1 byte via interrupt

    // Step 1: Enter AT command mode by sending "+++"
    const char enter_cmd[] = "+++";
    volatile char ok_enter[4] = {0}; 
    send_AT_cmd(enter_cmd, 3, ok_enter, 3, 20);

    // Step 2: Check its an "OK" response
    if( (ok_enter[0] == 'O') && (ok_enter[1] == 'K') && (ok_enter[2] == '\r') ){/*all good*/}
    else{return;} // get out

    // Step 3: Actually get the data we want :)
    const char cmd1[] = "ATNI\r";
    volatile char name[8] = {0}; // Empty arr to take name
    send_AT_cmd(cmd1, 5, name, 7, 30);
    name[6] = '\n'; 
    name[7] = '\r'; 

    // Step 4: GET OUT OF CMD MODE
    const char exit_cmd[] = "ATCN\r";
    volatile char ok_exit[4] = {0}; 
    send_AT_cmd(exit_cmd, 5, ok_exit, 3, 50);

    // Step 5: Check its an "OK" response
    if( (ok_enter[0] == 'O') && (ok_enter[1] == 'K') && (ok_enter[2] == '\r') ){/*all good*/}
    else{HAL_Delay(10100);} // No ok? ==> Delay 10.1 seconds to exit CMD mode
    
    // Step 6: Send the data so it can be wirelessly transmited
    HAL_UART_Transmit(&huart2, name, sizeof(name), HAL_MAX_DELAY);
    return;
}

// rx_length INCLUDES the \r at the end of the msg 
void send_AT_cmd(char* tx_msg, uint32_t tx_length, char* rx_msg, uint32_t rx_length, uint32_t delay){

    // 1) Transmit the msg 
    for (int i = 0; i < tx_length; i++) {
        HAL_Delay(20);
        HAL_UART_Transmit(&huart2, (uint8_t *)&tx_msg[i], 1, HAL_MAX_DELAY);
    }

    // 2) Receive the msg
    volatile char RXdata = 0; // start as invalid value
    uint32_t index = 0;
    uint32_t iterations = 0;
    const uint32_t MAX_ITERATIONS = 48000000;  // ~15 sec
    while(1){
        // Break when collected rx_length bytes
        if(index == rx_length){break;}

        // Break if reach max iterations
        if(iterations >= MAX_ITERATIONS){break;}
        iterations++;

        // Collect data bytes
        if(UART_FIFO_get_Char(&RXdata)){
          rx_msg[index] = RXdata;
          index ++;
        }
    }

}



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
