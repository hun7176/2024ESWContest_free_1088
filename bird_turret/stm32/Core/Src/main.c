/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "cmsis_os.h"
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

ETH_TxPacketConfig TxConfig;
ETH_DMADescTypeDef
    DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef
    DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

ETH_HandleTypeDef heth;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

osThreadId uartTaskHandle;
osSemaphoreId cooldownSemHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_UART5_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
void StartUartTask(void const *argument);

static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int8_t rxbuf;
int8_t uartRxData[4];
UART_HandleTypeDef *huartn = &huart5;
const uint8_t RES_ERR = 0, RES_DONE = 1;
int __io_putchar(int ch) {
  HAL_UART_Transmit(huartn, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}
int __io_getchar() {
  uint8_t ch = 0;
  /* Clear the Overrun flag just before receiving the first character */
  __HAL_UART_CLEAR_OREFLAG(huartn);
  /* Wait for reception of a character on the USART RX line and echo this
   * character on console */
  HAL_UART_Receive(huartn, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  HAL_UART_Transmit(huartn, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_UART5_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  // HAL_UART_Receive_IT(huartn, (uint8_t *)&dir, 1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  HAL_UART_Receive_IT(huartn, &rxbuf, 1);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of cooldownSem */
  osSemaphoreDef(cooldownSem);
  cooldownSemHandle = osSemaphoreCreate(osSemaphore(cooldownSem), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of uartTask */
  osThreadDef(uartTask, StartUartTask, osPriorityNormal, 0, 128);
  uartTaskHandle = osThreadCreate(osThread(uartTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
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
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief NVIC Configuration.
 * @retval None
 */
static void MX_NVIC_Init(void) {
  /* UART5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(UART5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(UART5_IRQn);
  /* USART3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

/**
 * @brief ETH Initialization Function
 * @param None
 * @retval None
 */
static void MX_ETH_Init(void) {

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

  static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK) {
    Error_Handler();
  }

  memset(&TxConfig, 0, sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes =
      ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 2800 - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 30000 - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 280 - 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 6000 - 1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 450 - 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);
}

/**
 * @brief UART5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART5_Init(void) {

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
  if (HAL_UART_Init(&huart5) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void) {

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */
}

/**
 * @brief USB_OTG_FS Initialization Function
 * @param None
 * @retval None
 */
static void MX_USB_OTG_FS_PCD_Init(void) {

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 4;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin | LD3_Pin | LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3 | USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin | LD3_Pin | LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PG3 USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_3 | USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// setmotor
#define MOVEOP 0
#define TRIGOP 1
#define ENDOFDATA 2
#define PHIMAX 750
#define PHIMIN 150
#define THTMAX 750
#define THTMIN 450
const int PHICENTER = 450 - 1, THTCENTER = 740 - 1;
volatile int phipulse = PHICENTER, thtpulse = THTCENTER;
volatile int cooldown = 0, received = 0;
// volatile float prevphierr = 0.0, prevthterr = 0.0;
// volatile float integphi = 0.0, integtht = 0.0;
// volatile float diffphi = 0.0, difftht = 0.0;
// const float dt = 1.0 / 30;
const float kpx = .13, kix = .01, kdx = .05;
const float kpy = .13, kiy = .01, kdy = .05;
const int BOUNDCOUNT = 30;
volatile int bounderr = BOUNDCOUNT;
const int DFLTPULSE = 210, TRIGPULSE = 680;
void SetMotor(int8_t nowphierr, int8_t nowthterr) {
  // pid control
  // integphi += nowphierr;
  // integtht += nowthterr;
  // diffphi = (nowphierr - prevphierr);
  // difftht = (nowthterr - prevthterr);
  // phipulse += kpx * nowphierr + kix * integphi + kpx * diffphi;
  // thtpulse += kpy * nowthterr + kiy * integtht + kpy * difftht;
  phipulse += kpx * nowphierr;
  thtpulse += kpy * nowthterr;

  // check the bound
  // phipulse = phipulse > PHIMIN ? phipulse : PHIMIN;
  // phipulse = phipulse < PHIMAX ? phipulse : PHIMAX;
  // thtpulse = thtpulse > THTMIN ? thtpulse : THTMIN;
  // thtpulse = thtpulse < THTMAX ? thtpulse : THTMAX;
  // if (phipulse == PHIMIN || phipulse == PHIMAX || thtpulse == THTMIN ||
  //     thtpulse == THTMAX)
  //   bounderr--;
  // if (!bounderr) {
  //   phipulse = PHICENTER, thtpulse = THTCENTER;
  //   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
  //   HAL_UART_Transmit(huartn, &RES_ERR, 1, HAL_MAX_DELAY);
  //   bounderr = BOUNDCOUNT;
  //   phipulse = PHICENTER, thtpulse = THTCENTER;
  //   // prevphierr = 0.0, prevthterr = 0.0;
  //   // integphi = 0.0, integtht = 0.0;
  //   // diffphi = 0.0, difftht = 0.0;
  // }

  // set duty cycle
  TIM3->CCR3 = thtpulse;
  TIM3->CCR4 = phipulse;
  // prevphierr = nowphierr, prevthterr = nowthterr;
}

// DoTrigger
void DoTrigger() {
  TIM3->CCR1 = TRIGPULSE;
  osDelay(500);
  TIM3->CCR1 = DFLTPULSE;
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
  osDelay(500);
  TIM3->CCR1 = TRIGPULSE;
  osDelay(500);
  TIM3->CCR1 = DFLTPULSE;
  osDelay(500);
  bounderr = BOUNDCOUNT;
  TIM3->CCR3 = THTCENTER, TIM3->CCR4 = PHICENTER;
  // prevphierr = 0.0, prevthterr = 0.0;
  // nowphierr = 0.0, nowthterr = 0.0;
  // integphi = 0.0, integtht = 0.0;
  // diffphi = 0.0, difftht = 0.0;
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
  HAL_UART_Transmit(huartn, &RES_DONE, 1, HAL_MAX_DELAY);
  cooldown = 1;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  uartRxData[0] = uartRxData[1];
  uartRxData[1] = uartRxData[2];
  uartRxData[2] = uartRxData[3];
  uartRxData[3] = rxbuf;
  if (rxbuf == ENDOFDATA)
    received = 1;
  HAL_UART_Receive_IT(huartn, &rxbuf, 1);
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartUartTask */
/**
 * @brief  Function implementing the uartTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartUartTask */
void StartUartTask(void const *argument) {
  /* USER CODE BEGIN 5 */
  TIM3->CCR3 = THTCENTER;
  TIM3->CCR4 = PHICENTER;
  TIM3->CCR1 = DFLTPULSE;
  /* Infinite loop */
  for (;;) {
    HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_SET);
    while (!received) {
      osDelay(1);
    }
    HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_RESET);
    received = 0;
    if (uartRxData[0] == MOVEOP) {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
      // SetMotor(-uartRxData[1], uartRxData[2]);
      phipulse += kpx * (-uartRxData[1]);
      thtpulse += kpy * uartRxData[2];
      TIM3->CCR3 = thtpulse;
      TIM3->CCR4 = phipulse;
    } else if (uartRxData[0] == TRIGOP) {
      DoTrigger();
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    }
    HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_SET);
    HAL_Delay(1000);
    HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_RESET);
  }
  /* USER CODE END 5 */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  /* USER CODE BEGIN Callback 0 */
  if (htim->Instance == TIM2) {
    osSemaphoreWait(cooldownSemHandle, osWaitForever);
    cooldown = 0;
    osSemaphoreRelease(cooldownSemHandle);
  }
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
