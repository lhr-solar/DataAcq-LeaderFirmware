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

// Temporary storage for reading CAN msgs
typedef struct __attribute__((__packed__)) {
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];
} CAN_UART_Packet;

// MATTHEW formated data transmision
typedef struct __attribute__((__packed__)) {
    char     delimiter;   // ascii delimiter
    uint16_t can_id;      // 0 to 0x7FF
    uint16_t time_stamp;  // 10ms period (tim2)
    uint8_t  num_bytes;   // 0 to 8 bytes of data
    uint8_t  data[8];     // data payload
} CAN_FORMATTED_Packet;

#define SLCAN_MAX_STRING_LEN 32  // Enough for 't' + 3 (id) + 1 (dlc) + 16 (data) + 1 (\r) + 1 (\0)
typedef struct {
    const char* name;
    uint16_t can_id;   // ASCII version of CAN ID
    uint8_t  length;       // in bytes
    uint8_t  index_used;   // 1 if index used, 0 otherwise
} CANSignal;

// SLCAN formated data transmision
typedef struct {
    char slcanmsg[SLCAN_MAX_STRING_LEN];
} SLCAN;

// AT Command (send, and receive)
#define AT_CMD_MAX_LEN 16      // AT CMD = 4, \r\0 = 2, ==> 6
#define RX_MSG_MAX_LEN 16     // 16 ASCII Hex Chars = 8 bytes = Max CAN data payload
typedef struct {
    char id[4];               // Can ID (null terimating string)
    char tx[AT_CMD_MAX_LEN];  // Must terminate with \r
    uint8_t tx_len;           // Number of chars transmiting (including the \r)
    char rx[RX_MSG_MAX_LEN];  // Will terminate with \r
    uint8_t rx_len;           // How many bytes were recieved 
    uint8_t rx_max_bytes;     // Max number of bytes for this can msg
} AT_CMD;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))
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

// For CAN1 Reception (slcan)
// -----------------------------
#define CAN_FIFO_SIZE 1024 // 256 SL-CAN Msgs

volatile SLCAN rf_tx_fifo[CAN_FIFO_SIZE];
volatile uint16_t rf_tx_fifo_head = 0;
volatile uint16_t rf_tx_fifo_tail = 0;

volatile SLCAN lte_tx_fifo[CAN_FIFO_SIZE];
volatile uint16_t lte_tx_fifo_head = 0;
volatile uint16_t lte_tx_fifo_tail = 0;
// -----------------------------

// SLCAN shit 
// ------------------------------------------------------------------------------------------------
CANSignal can_signals[] = {
    // Controls.csv
    { "CONTROL_MODE",                         0x580,  1, 0 },
    { "Motor Precharge Enable",               0x582,  1, 0 },
    { "IO_STATE",                             0x581,  3, 0 },
    { "Controls_Fault",                       0x583,  1, 0 },
    { "Motor Controller Safe",                0x584,  1, 0 },
    { "Motor Controller Identification",      0x240,  8, 0 },
    { "Motor Status",                         0x241,  8, 0 },
    { "Motor Controller Bus",                 0x242,  8, 0 },
    { "Velocity",                             0x243,  8, 0 },
    { "Motor Controller Phase Current",       0x244,  8, 0 },
    { "Motor Voltage Vector",                 0x245,  8, 0 },
    { "Motor Current Vector",                 0x246,  8, 0 },
    { "Motor BackEMF",                        0x247,  8, 0 },
    { "Low Voltage Rail Measurement",         0x248,  8, 0 },
    { "DSP Voltage Rail Measurement",         0x249,  8, 0 },
    { "Reserved",                             0x24A,  8, 0 },
    { "Motor Temperature",                    0x24B,  8, 0 },
    { "DSP Board Temperature",                0x24C,  8, 0 },
    { "Reserved",                             0x24D,  8, 0 },
    { "Odometer / Bus Amp Hours",             0x24E,  8, 0 },
    { "Slip Speed Measurement",               0x257,  8, 0 },
    // Shared.csv
    { "Dash Kill Switch",                     0x001,  1, 0 },
    { "Any System Failures",                  0x003,  1, 0 },
    { "Ignition",                             0x004,  1, 0 },
    { "Any System Shutoff",                   0x005,  1, 0 },
    // BPS.CSV
    { "BPS Trip",                             0x002,  1, 0 },
    { "BPS All Clear",                        0x101,  1, 0 },
    { "BPS Contactor State",                  0x102,  1, 0 },
    { "Current Data",                         0x103,  4, 0 },
    { "Voltage Data Array",                   0x104,  5, 1 },
    { "Temperature Data Array",               0x105,  5, 1 },
    { "State of Charge Data",                 0x106,  4, 0 },
    { "WDog Triggered",                       0x107,  1, 0 },
    { "CAN Error",                            0x108,  1, 0 },
    { "BPS Command msg",                      0x109,  8, 0 },
    { "Supplemental Voltage",                 0x10B,  2, 0 },
    { "Charging Enabled",                     0x10C,  1, 0 },
    { "Voltage Summary",                      0x10D,  8, 0 },
    { "Temperature Summary",                  0x10E,  8, 0 },
    { "BPS Fault State",                      0x10F,  1, 0 },
    // Blackbody.csv
    { "Heartbeat (BPS)",                      0x650,  1, 0 },
    { "Set Mode (BPS)",                       0x651,  1, 0 },
    { "Board Fault (MPPT)",                   0x652,  2, 0 },
    { "Acknowledge Fault (MPPT)",             0x653,  1, 0 },
    { "Temperature Sensor Configure",         0x654,  3, 0 },
    { "Irradiance Sensor Configure",          0x655,  3, 0 },
    { "Temperature Measurement",              0x656,  5, 0 },
    { "Irradiance Measurement",               0x657,  5, 0 },
    // Contactors.csv
    { "Contactor Sense",                      0x400,  2, 0 },
    { "Precharge Timeout",                    0x401,  1, 0 },
    // Sunscatter.csv
    { "Heartbeat (Controller)",               0x600,  1, 0 },
    { "Set Mode; BOARD OVERRIDE ENABLE",      0x601,  1, 0 },
    { "Board Fault (Controller)",             0x602,  2, 0 },
    { "Acknowledge Fault (Controller)",       0x603,  1, 0 },
    { "Sensor Configure",                     0x604,  5, 0 },
    { "Sensor Configure 2",                   0x605,  5, 0 },
    { "Sensor Configure 3",                   0x606,  5, 0 },
    { "Controller Configure",                 0x607,  0, 0 },
    { "Debug Configure",                      0x608,  1, 0 },
    { "Operating Setpoint",                   0x609,  8, 0 },
    { "Input Voltage Measurement",            0x60A,  4, 0 },
    { "Input Current Measurement",            0x60B,  4, 0 },
    { "Output Voltage Measurement",           0x60C,  4, 0 },
    { "Output Current Measurement",           0x60D,  4, 0 },
    // TPEE.csv
    { "MPPT A Status",                        0x201,  5, 0 },
    { "MPPT A Boost Enable",                  0x209,  1, 0 },
    { "MPPT B Status",                        0x211,  5, 0 },
    { "MPPT B Boost Enable",                  0x219,  1, 0 }
};

#define CAN_SIGNAL_COUNT ARRAY_SIZE(can_signals) 
// ------------------------------------------------------------------------------------------------


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

  // Read Either LTE or RF MetaData from module (called by user) 
  void read_meta_data(UART_HandleTypeDef* huart_ptr, AT_CMD* list_AT_CMDs, uint32_t num_cmds);

  // Sends an AT Command to LTE or RF Module (helper function of read_meta_data)
  void send_AT_cmd(UART_HandleTypeDef* huart_ptr, AT_CMD* at_cmd);

  // Formats a char* data packet into MATTHEW data transmit format. Used for data acq internal can msgs (called by user)  
  void format_UART_Msg(char* msg, CAN_FORMATTED_Packet* formatted_msg);

  // Turns (temporary) CAN Packet format into MATTHEW data transmit format. (not being called rn)
  void parse_RX_CAN(CAN_UART_Packet* rx_msg, CAN_FORMATTED_Packet* formatted_msg, uint16_t time_stamp);

  // Debugging function to transmit MATTHEW data format in a more readable format (called by user ig)
  void transmit_ASCII_CAN_Packet(CAN_FORMATTED_Packet* formatted_msg);

  // Takes CAN data and turns it into ASCII SLCAN Format (called in CAN1 ISR & in some testing stuff)
  uint32_t format_slcan_frame(uint16_t can_id, uint8_t* data, uint8_t dlc, char* out_str);

  // Takes AT_CMD data and turns it into ASCII SLCAN Format (helper function for AT_CMDs)
  // uint32_t format_slcan_frame_AT_CMD(char* can_id, char* data, uint8_t rx_bytes, char* out_str);
  uint32_t format_slcan_frame_AT_CMD(AT_CMD* command, char* out_str);

  // Helper function that determines char* length by finding the \r 
  uint32_t tx_msg_len(char* tx_msg);

  // Helper function of send_at_cmd(). UART ISR puts RX bytes into this FIFO, send_at_cmd() reads them out
  int UART_FIFO_get_Char(char* rx_char);

  // Generic push and pop for "Transmit" SW FIFO (can be any sw fifo of type SLCAN)
  uint8_t tx_fifo_push(SLCAN* tx_fifo, uint16_t* tx_fifo_head, uint16_t* tx_fifo_tail, char* msg);
  uint8_t tx_fifo_pop(SLCAN* tx_fifo, uint16_t* tx_fifo_head, uint16_t* tx_fifo_tail, SLCAN* poped_msg);

  // Function to compute and send fifo element count  
  void send_fifo_count(uint16_t can_id, uint16_t head, uint16_t tail);

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

// Failed to Send UART (RF, LTE)                      2
// Failed to format to SLCAN (at cmd, normal)         2
// HW FIFO is full (RF, LTE)                          2 
// SW FIFO is full (RF, LTE, UART)                    3
// Failed to read msg out of HW fifo (Uart, CAN)      2
// Didnt get an OK from readmeta data (enter, exit)   2
// Total of 13 Error codes 
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

  // CAN Filter: 
  // ----------------------------------------------
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
  // ----------------------------------------------

  // Start CAN1
  HAL_CAN_Start(&hcan1);

  // Start TIMs
  HAL_TIM_Base_Start(&htim14);
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_Base_Start(&htim5);

  // NOTE: ############ Very important for UART2 ISR reception ################
  __enable_irq();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // Set Priority level and enable IRS for UART2 RX 
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);

  // Set Priority level and enable IRS for UART5 RX 
  HAL_NVIC_SetPriority(UART5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(UART5_IRQn);

  // Set Priority level and enable IRS for CAN1 RX 
  HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    // Note: "RX0" == Use FIFO0 (there is also a FIFO1)

  // Enable RX interrupt
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  // HAL_Delay(2000);
  while (1)
  {

      // // MODE #1) Infinate SLCAN to Xbee LTE: (have tested)
      // // #########################################################################################################
      // char slcan_str2[SLCAN_MAX_STRING_LEN] = {0};
      // uint8_t dummy_data;
      // while (1) {
      //     // Go through all CAN ID and send dummy data
      //     for (size_t i = 0; i < CAN_SIGNAL_COUNT; i++) {
      //         // Flow control: 
      //         GPIO_PinState NCTS = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9);
      //         if (NCTS == GPIO_PIN_SET) {i--; cyc();}  // NCTS = 1 means NO SENDING. reset i backwards 1
      //         else{
      //             // Make up dummy_data:
      //             volatile uint32_t timer_val = __HAL_TIM_GET_COUNTER(&htim2);
      //             dummy_data = ((timer_val >> (i % 8)) ^ (i * 37)) & 0xFF;

      //             // Format into SLCAN: 
      //             format_slcan_frame(can_signals[i].can_id, &dummy_data, 1, slcan_str2);

      //             // Send and stall 
      //             HAL_UART_Transmit(&huart5, (uint8_t *)slcan_str2, tx_msg_len(slcan_str2), HAL_MAX_DELAY);
      //             HAL_Delay(1000);
      //         }
      //     }
      // }
      // // #########################################################################################################


      // MODE #2) CAN1 + Meta Data -> SLCAN FIFO -> XBee RF + LTE:
      // #########################################################################################################

      // GC, GT is the guard time (rn its 1 sec, can change)
        // AT_GT_5_\r // Changes guard time to 5
        // Dont need to do anything specil, just exit CMD mode 
      // CT is how long it waits before dropping the Command mode


      // RF AT Commands List
      AT_CMD RF_AT_CMDs[] = {
        //{ .id = "700",                                                },   // RF SW Fifo Element Count 
          { .id = "701", .tx = "ATBC\r", .tx_len = 5, .rx_max_bytes = 4 },   // Bytes Transmited
          { .id = "702", .tx = "ATTR\r", .tx_len = 5, .rx_max_bytes = 2 },   // Transmision Failer Count
          { .id = "703", .tx = "ATDB\r", .tx_len = 5, .rx_max_bytes = 1 },   // Last Packet RSSI
          { .id = "704", .tx = "ATGD\r", .tx_len = 5, .rx_max_bytes = 2 },   // Good Packet Received
          { .id = "705", .tx = "ATEA\r", .tx_len = 5, .rx_max_bytes = 2 },   // MAC ACK Failer Count
          { .id = "706", .tx = "ATGT\r", .tx_len = 5, .rx_max_bytes = 2 },   // Guard Time
          { .id = "707", .tx = "ATCT\r", .tx_len = 5, .rx_max_bytes = 1 },   // Command Mode Timeout 
      };
      // LTE At Commands List
      AT_CMD LTE_AT_CMDs[] = {
        //{ .id = "780",                                                },   // LTE SW Fifo Element Count 
          { .id = "781", .tx = "ATDB\r", .tx_len = 5, .rx_max_bytes = 1 },   // Cellular Singal Strength
          { .id = "782", .tx = "ATGT\r", .tx_len = 5, .rx_max_bytes = 2 },   // Guard Time 
          { .id = "783", .tx = "ATCT\r", .tx_len = 5, .rx_max_bytes = 1 },   // Command Mode Timeout 
          // { .id = "381", .tx = "ATFC\r" },   // Freq Channel Number
          // { .id = "382", .tx = "ATDT\r" },   // Time UTC
      };
      // Changing the Guard Time & Timeout Time
      AT_CMD module_configure[] = {
          { .id = "XXX", .tx = "ATGT2\r", .tx_len = 6, .rx_max_bytes = 0 },   // Guard Time = 2ms (min)
          { .id = "XXX", .tx = "ATCT2\r", .tx_len = 6, .rx_max_bytes = 0 },   // Command Mode Timeout = 200ms (min) 
          { .id = "XXX", .tx = "ATWR\r",  .tx_len = 5, .rx_max_bytes = 0 },    // Write the changes to non volatile flash mem 
      };

      // Super Loop
      uint32_t iterations = 0;
      uint32_t lte_iterations = 0;
      const uint32_t MAX_ITERATIONS = 9600000;  // ~12 sec when no CAN msgs
      for(int i = 0; i < 10; i++){cyc(); HAL_Delay(100);}
      while(1) {

          // Configure the Guard Time and Timeout Time
          // -----------------------------------------------------------------------------------------------------
          // RF:
          // read_meta_data(&huart2, &module_configure, ARRAY_SIZE(module_configure));
          // LTE:
          // read_meta_data(&huart5, &module_configure, ARRAY_SIZE(module_configure));
          // NOTE NOTE NOTE NOTE NOTE NOTE 
          // Need to comment out the tx_push part of the read_meta_data outherwise you WILL HARDFAULT 
          // ALSO comment back in the hal delay in there (has a comment next to it)
          // while(1){
          //   HAL_Delay(50);
          //   cyc();
          // }
          // -----------------------------------------------------------------------------------------------------

          // Attempt to send LTE (have not tested)
          // -----------------------------------------------------------------------------------------------------
          GPIO_PinState LTE_NCTS = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9);
          if (LTE_NCTS == GPIO_PIN_SET) {cyc();}  // HW FIFO Full
          else{
              SLCAN poped_msg;
              if(tx_fifo_pop(&lte_tx_fifo, &lte_tx_fifo_head, &lte_tx_fifo_tail, &poped_msg)){
                  if(HAL_UART_Transmit(&huart5, (uint8_t *)poped_msg.slcanmsg, tx_msg_len(poped_msg.slcanmsg), HAL_MAX_DELAY) != HAL_OK){cyc(); /* UART Fail */}
                  if(lte_iterations > 10){
                    send_fifo_count(0x780, lte_tx_fifo_head, lte_tx_fifo_tail);     // Num elements in LTE SW Fifo = id 0x780
                    lte_iterations = 0;
                  }
                  else{lte_iterations ++;}
                  // NOTE: right now if this happens the tx msg is just discarded
              }
          }
          // -----------------------------------------------------------------------------------------------------

          // Attempt to send RF (tested)
          // -----------------------------------------------------------------------------------------------------
          GPIO_PinState RF_NCTS = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);
          if (RF_NCTS == GPIO_PIN_SET) {cyc();} 
          else{
              SLCAN poped_msg;
              if(tx_fifo_pop(&rf_tx_fifo, &rf_tx_fifo_head, &rf_tx_fifo_tail, &poped_msg)){
                  if(HAL_UART_Transmit(&huart2, (uint8_t *)poped_msg.slcanmsg, tx_msg_len(poped_msg.slcanmsg), HAL_MAX_DELAY) != HAL_OK){cyc(); /* UART Fail */}
                  send_fifo_count(0x700, rf_tx_fifo_head, rf_tx_fifo_tail);       // Num elements in RF SW Fifo = id 0x700
                  // NOTE: right now if this happens the tx msg is just discarded
              }
          }
          // -----------------------------------------------------------------------------------------------------

          // Read / Compute & Transmit Meta Data
          // -----------------------------------------------------------------------------------------------------
          if(iterations >= MAX_ITERATIONS){
            read_meta_data(&huart2, &RF_AT_CMDs, ARRAY_SIZE(RF_AT_CMDs));
            read_meta_data(&huart5, &LTE_AT_CMDs, ARRAY_SIZE(LTE_AT_CMDs));
            iterations = 0;
          }
          else{iterations ++;}
          // -----------------------------------------------------------------------------------------------------
      }
      // #########################################################################################################
      
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

  // RF NCTS Flow Control
  // Now, configure PA8 as input (separate setup)
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP; // Default to 1 = NO SENDING (prob safer = no data loss) 
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // LTE NCTS Flow Control
  // Now, configure PA9 as input (separate setup)
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP; // Default to 1 = NO SENDING (prob safer = no data loss) 
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// Finds the number of elements in the SW fifo
// Head and tails are indexes into the FIFO
// Then formats into SLCAN format (with provided canID) and puts into both RF and LTE sw fifos
void send_fifo_count(uint16_t can_id, uint16_t head, uint16_t tail){
    uint16_t count = 0;
    if (head >= tail) {count = head - tail;} 
    else {count = CAN_FIFO_SIZE - tail + head;}

    char slcan_msg [SLCAN_MAX_STRING_LEN];
    // uint32_t format_slcan_frame(uint16_t can_id, uint8_t* data, uint8_t dlc, char* out_str) {
    if(!format_slcan_frame(can_id, (uint8_t*)&count, 2, &slcan_msg)){cyc(); return;} // Failed to format

    uint8_t rf_push_success  = 1;
    uint8_t lte_push_success = 1;

    __disable_irq();  
    rf_push_success &= tx_fifo_push(rf_tx_fifo, &rf_tx_fifo_head, &rf_tx_fifo_tail, slcan_msg);   // RF Transmit
    lte_push_success &= tx_fifo_push(lte_tx_fifo, &lte_tx_fifo_head, &lte_tx_fifo_tail, slcan_msg);// LTE Transmit
    __enable_irq();

    if(!rf_push_success){cyc();}
    if(!lte_push_success){cyc();}
}


// Push a message into the CAN FIFO.
// Assumes caller disables interrupts if needed.
// Returns 1 if successful, 0 if buffer is full.
uint8_t tx_fifo_push(SLCAN* tx_fifo, uint16_t* tx_fifo_head, uint16_t* tx_fifo_tail, char* msg){

    // Error checking
    if (msg == NULL || tx_fifo == NULL || tx_fifo_head == NULL || tx_fifo_tail == NULL || tx_fifo[*tx_fifo_head].slcanmsg == NULL){
        return 0; 
    }

    uint16_t next_head = (*tx_fifo_head + 1) % CAN_FIFO_SIZE;
    if (next_head == *tx_fifo_tail) {
        return 0;  // FIFO full
    }

    // Deep copy into SW FIFO
    strcpy(tx_fifo[*tx_fifo_head].slcanmsg, msg);
    *tx_fifo_head = next_head;

    return 1;  // Success
}

// Pop from the provided SW fifo 
uint8_t tx_fifo_pop(SLCAN* tx_fifo, uint16_t* tx_fifo_head, uint16_t* tx_fifo_tail, SLCAN* poped_msg) {
    if (*tx_fifo_head == *tx_fifo_tail) {
        // Buffer is empty
        return 0;
    }

    strcpy(poped_msg->slcanmsg, tx_fifo[*tx_fifo_tail].slcanmsg);
    *tx_fifo_tail = (*tx_fifo_tail + 1) % CAN_FIFO_SIZE;
    return 1;
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    // Values populated by HAL_CAN_GetRxMessage:
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];

    // Get Data out of CAN HW FIFO
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK){cyc(); return;}

    // Format data as slcan 
    volatile char slcan_msg [SLCAN_MAX_STRING_LEN];
    if(!format_slcan_frame(rxHeader.StdId, rxData, rxHeader.DLC, &slcan_msg)){cyc(); return;} // Failed to format

    // Push slcan_msg into SW FIFO 
    uint8_t success = 1;
    success &= tx_fifo_push(rf_tx_fifo, &rf_tx_fifo_head, &rf_tx_fifo_tail, slcan_msg);   // RF Transmit
    success &= tx_fifo_push(lte_tx_fifo, &lte_tx_fifo_head, &lte_tx_fifo_tail, slcan_msg);// LTE Transmit
    if(!success){cyc();} // Push failed, SW FIFO Full
}

void CAN1_RX0_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&hcan1); // HAL handles it, then calls your callback
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if ( (huart->Instance == USART2) || (huart->Instance == UART5)) {
        // Put RX byte into circular SW FIFO
        uint16_t next_head = (fifo_head + 1) % FIFO_SIZE;
        if (next_head != fifo_tail) {  // Check for overflow
            fifo[fifo_head] = rx_buffer;
            fifo_head = next_head;
        } else {
          cyc();
        }
        // Re-enable interrupt for next byte
        HAL_UART_Receive_IT(huart, &rx_buffer, 1);
    }
}

void UART5_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart5);
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

void read_meta_data(UART_HandleTypeDef* huart_ptr, AT_CMD* list_AT_CMDs, uint32_t num_cmds){
    // XBee needs 1 second of silence before and after sending "+++"
    // HAL_Delay(1100); 
    HAL_Delay(20); 

    // Step 0: Arm reception of UART data
    HAL_UART_Receive_IT(huart_ptr, &rx_buffer, 1);

    // Step 1: Enter AT command mode by sending "+++"
    AT_CMD enter = {.tx = "+++", .tx_len = 3, .rx = {0}, .rx_len = 0};
    send_AT_cmd(huart_ptr, &enter);

    // Step 2: Check its an "OK" response
    if( (enter.rx[0] == 'O') && (enter.rx[1] == 'K') && (enter.rx[2] == '\r') ){/*all good*/}
    else{cyc(); return;} // get out

    // Step 3: Actually get the data we want :)
    for(int i = 0; i < num_cmds; i ++){
        send_AT_cmd(huart_ptr, &(list_AT_CMDs[i]));
    }

    // COMMENT THIS IN/OUT IF YOU WANT TO CONFIGURE SETTINGS (commented out = normal operation)
    // HAL_Delay(500);

    // Step 4: GET OUT OF CMD MODE
    AT_CMD exit = {.tx = "ATCN\r", .tx_len = 5, .rx = {0}, .rx_len = 0};
    send_AT_cmd(huart_ptr, &exit);

    // Step 5: Check its an "OK" response
    if( (exit.rx[0] == 'O') && (exit.rx[1] == 'K') && (exit.rx[2] == '\r') ){/*all good*/}
    // else{cyc(); HAL_Delay(10100);} // No ok? ==> Delay 10.1 seconds to exit CMD mode
    else{cyc(); HAL_Delay(220);} // No ok? ==> Delay 10.1 seconds to exit CMD mode
    
    // Step 6: Add the data to SW FIFOs
    char slcan_msg [SLCAN_MAX_STRING_LEN];
    for(int i = 0; i < num_cmds; i ++){
        // if(format_slcan_frame_AT_CMD(list_AT_CMDs[i].id, list_AT_CMDs[i].rx, list_AT_CMDs[i].rx_len, slcan_msg)){
        if(format_slcan_frame_AT_CMD(&(list_AT_CMDs[i]), slcan_msg)){
            // ISR also pushes to can fifo, so need to make atomic
            uint8_t success = 1;
            __disable_irq();  
            success &= tx_fifo_push(rf_tx_fifo, &rf_tx_fifo_head, &rf_tx_fifo_tail, slcan_msg);   // RF Transmit
            success &= tx_fifo_push(lte_tx_fifo, &lte_tx_fifo_head, &lte_tx_fifo_tail, slcan_msg);// LTE Transmit
            __enable_irq();
            if(!success){cyc();} // FIFO Full = cyc()
        }
    }
    return;
}

// rx_length INCLUDES the \r at the end of the msg 
void send_AT_cmd(UART_HandleTypeDef* huart_ptr, AT_CMD* at_cmd){

    // 1) Transmit the msg 
    // uint8_t len_tx = 5; // Hanlde "+++" edge case (len of 3 not 5)
    // if((at_cmd->tx[0] == '+') && (at_cmd->tx[1] == '+') && (at_cmd->tx[2] == '+')){len_tx = 3;}
    for (int i = 0; i < at_cmd->tx_len; i++) {
        HAL_UART_Transmit(huart_ptr, (uint8_t *)&at_cmd->tx[i], 1, HAL_MAX_DELAY);
    }

    // 2) Receive the msg
    volatile char RXdata = 0; // start as invalid value
    uint32_t index = 0;
    uint32_t iterations = 0;
    // const uint32_t MAX_ITERATIONS = 4800000;  // 1.5 sec
    const uint32_t MAX_ITERATIONS = 640000;  // ~ 200 ms 
    while(1){
        // Collect data bytes
        if(UART_FIFO_get_Char(&RXdata)){
          at_cmd->rx[index] = RXdata;
          index ++;
        }

        // Break if got a '\r' (after adding it to rx_msg) OR if reach max iterations
        if( (RXdata == '\r') || (iterations >= MAX_ITERATIONS)){break;}
        iterations++;
    }

    // Update Rx_len field of AT_CMD 
    at_cmd->rx_len = index; // (includes the \r in len count)
}



// // AT Command (send, and receive)
// #define AT_CMD_MAX_LEN 6      // AT CMD = 4, \r\0 = 2, ==> 6
// #define RX_MSG_MAX_LEN 16     // 16 ASCII Hex Chars = 8 bytes = Max CAN data payload
// typedef struct {
//     char id[4];               // Can ID (null terimating string)
//     char tx[AT_CMD_MAX_LEN];  // Must terminate with \r
//     char rx[RX_MSG_MAX_LEN];  // Will terminate with \r
//     uint8_t rx_len;           // How many bytes were recieved 
//     uint8_t rx_max_bytes;     // Max number of bytes for this can msg
// } AT_CMD;


// Assumes that DLC is uint8_t not char 
// uint32_t format_slcan_frame_AT_CMD(char* can_id, char* data, uint8_t rx_bytes, char* out_str){
uint32_t format_slcan_frame_AT_CMD(AT_CMD* command, char* out_str){
    char* can_id       = command->id;
    char* data         = command->rx;
    uint8_t rx_len     = command->rx_len;
    uint8_t rx_max_len = command->rx_max_bytes;

    if (rx_len > 8 || out_str == NULL || data == NULL) {
        strcpy(out_str, ""); // Clear output on invalid input
        return 0;
    }

    // Start the frame with 't' and CAN ID
    // uint32_t num_bytes = rx_bytes/2;
    sprintf(out_str, "t%s%X", can_id, rx_max_len);
      // CAN msgs have fixed length (not dynamic)

    // Pad the front with zeros (if needed)
    uint8_t num_actual_data = rx_len - 1;
    uint8_t num_zero_padding = (2*rx_max_len) - num_actual_data;
    for(int i = 0; i < num_zero_padding; i ++){
      out_str[5 + i] = '0';
    }

    // Append data (already hex and has \r at end) 
    for(int i = 0; i < rx_len; i ++){
      out_str[5 + num_zero_padding + i] = data[i];
    }

    // Example of the padding:
      // Hello\r
      // Rx len = 6
      // Data Length = 5
      // 4 bytes ==> 8 Hex, 8-5 = 3 zeros 
      // ==> t_781_4_00_0H_el_lo_\r

    // Sucess 
    return 1;
}



// can_id   = [0, x7FF]
// data     = Max 8 bytes 
// dlc      = num bytes of data 
// out_str  = return str
// return   = 1 on success, 0 on fail
uint32_t format_slcan_frame(uint16_t can_id, uint8_t* data, uint8_t dlc, char* out_str) {
    if (can_id > 0x7FF || dlc > 8 || out_str == NULL) {
        strcpy(out_str, ""); // Clear output on invalid input
        return 0;
    }

    // Start the frame with 't' and CAN ID
    sprintf(out_str, "t%03X%X", can_id, dlc);

    // Append each data byte as two hex characters
    for (uint8_t i = 0; i < dlc; i++) {
        char byte_str[3];
        sprintf(byte_str, "%02X", data[i]);
        strcat(out_str, byte_str);
    }

    // Append carriage return
    strcat(out_str, "\r");

    // Sucess 
    return 1;
}

// Figure out the len of a TX MSG by looking for \r 
uint32_t tx_msg_len(char* tx_msg){
    uint32_t index = 0;
    while(1){
        if(tx_msg[index] == 0){return 0;}           // Failed to find \r
        if(tx_msg[index] == '\r'){return index+1;}  // Determined len
        index ++;
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
