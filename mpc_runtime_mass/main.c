/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MPC_timing.h"
#include <stdio.h>
#include <math.h>
#include "dma_printf.h"
#include "dma_ring.h"
#include "MPC_param.h"
#include "MPC_function.h"
#include "RIM_functions.h"
#include "function_3link.h"
#include "param_3link.h"
#include "io_3link.h"
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
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma location=0x2007c000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x2007c0a0
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x2007c000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x2007c0a0))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __GNUC__ ) /* GNU Compiler */

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */
#endif

ETH_TxPacketConfig TxConfig;

ETH_HandleTypeDef heth;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
/* 既存TIMを変更せず、MPC処理時間をDWTで観測する。 */
volatile int mpc_dwt_status = -999;
volatile uint32_t mpc_total_cycles = 0U;
volatile uint32_t mpc_total_time_us = 0U;
volatile uint32_t mpc_max_time_us = 0U;
volatile uint32_t mpc_overrun_count = 0U;


/* MPC parameters */
volatile double sensor_state[4] = {0.0, 0.0, 0.0, 0.0};
volatile int sensor_sample_ready = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM7_Init(void);
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
  /* CPUサイクルカウンタだけを準備する。TIM/CPU周波数は変更しない。 */
  mpc_dwt_status = MPC_CycleCounterInit();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM4_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  /*変数の初期化*/
  param_init();
  /* Encoder start Joint1 */
  HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL); //Encoder reading start Joint1
  HAL_TIM_Base_Start_IT(&htim3);//Encoder overflow countermeasure interrupt start

  /* Encoder start Joint2 */
  HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL); //Encoder reading start Joint2
  HAL_TIM_Base_Start_IT(&htim4);//Encoder overflow countermeasure interrupt start

  /* Encoder start Joint3 */
  HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_ALL); //Encoder reading start Joint3
  HAL_TIM_Base_Start_IT(&htim1);//Encoder overflow countermeasure interrupt start

  /* Control loop start */
  HAL_TIM_Base_Start_IT(&htim7);//Periodic Control

  /* Servo　ON*/
  HAL_GPIO_WritePin(On_Servo_GPIO_Port, On_Servo_Pin, GPIO_PIN_SET);
  overflowcnt[0]=0;
  overflowcnt[1]=0;
  overflowcnt[2]=0;

  //DMAの初期化
  setbuf(stdin, NULL);
  setbuf(stdout, NULL);
  setbuf(stderr, NULL);
  dma_printf_init(&huart3);//printfを使いたいUARTポートの構造体のポインタ

  //MPCの初期化
	/* ソルバの初期化と、起動時の入力制約更新を一度だけ実行する。 */
	if (MPC_Init() != 0)
	{
		printf("Error: MPC initialization failed: %d\r\n", (int)mpc_create_status);
		Error_Handler();
	}
	mpc_update_status = MPC_UpdateInputBounds(MPC_INPUT_MIN, MPC_INPUT_MAX);
	if (mpc_update_status != 0)
	{
		printf("Error: input bounds update failed: %d\r\n", (int)mpc_update_status);
		Error_Handler();
	}
	printf("MPC initialized. Waiting for valid EKF states, equivalent mass and contact condition.\r\n");


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

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

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 96-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 1000-1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 921600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
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
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(On_Servo_GPIO_Port, On_Servo_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, CS_DAC1_Pin|CS_DAC2_Pin|USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : On_Servo_Pin */
  GPIO_InitStruct.Pin = On_Servo_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(On_Servo_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_DAC1_Pin CS_DAC2_Pin USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = CS_DAC1_Pin|CS_DAC2_Pin|USB_PowerSwitchOn_Pin;
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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim){

	/* Control loop */
	if(htim->Instance == TIM7){
//		D_out[0] = 2048;
//		D_out[1] = 2048;
//		D_out[2] = 4096;

		if (t < t_end){

			/* th_1 (Joint1) */
			cnt_encoder[0] = (long)(overflowcnt[0]) * 0x10000 + (long)TIM3 -> CNT;
			th_res_deg[0] = 360.0f * (int)cnt_encoder[0] / (4.0f * 100.0f * 1000.0f);
			th_res[0] = th_res_deg[0] * pi / 180.0f;

			dth_res[0] = g_vm[0][0] * (th_res[0] - signal_dth_res[0]);
			signal_dth_res[0] += dth_res[0] * sampling_time;


			/* th_2 (Joint2) */
			cnt_encoder[1] = (long)(overflowcnt[1]) * 0x10000 + (long)TIM4 -> CNT;
			th_res_deg[1] = -360.0f * (int)cnt_encoder[1] / (4.0f * 1.0f * 1500.0f);
			th_res[1] = th_res_deg[1] * pi / 180.0f;

			dth_res[1] = g_vm[1][1] * (th_res[1] - signal_dth_res[1]);
			signal_dth_res[1] += dth_res[1] * sampling_time;

			/* th_3 (Joint3) */
			cnt_encoder[2] = (long)(overflowcnt[2]) * 0x10000 + (long)TIM1 -> CNT;
			th_res_deg[2] = -360.0f * (int)cnt_encoder[2] / (4.0f * 1.0f * 1500.0f);
			th_res[2] = th_res_deg[2] * pi / 180.0f;

			dth_res[2] = g_vm[2][2] * (th_res[2] - signal_dth_res[2]);
			signal_dth_res[2] += dth_res[2] * sampling_time;

		    //角度制御
			if(t<12&&t>5){
				if(flag_init==0){
					for(i=0;i<3;i++){
						i_a_ref[i] = 0.0f;
						 th_cmd[i] = 0.0f;
						 dth_cmd[i] = 0.0f;
						 ddth_ref[i] = 0.0f;
						 tau_ref[i] = 0.0f;
						 i_ref[i] = 0.0f;
						 signal_dob1[i] = 0.0f;
						 signal_dob2[i] = 0.0f;
						 hat_tau_dis[i] = 0.0f;
						 tau_cmd[i] = 0.0f;
						D_out[0] = 2048;
						D_out[1] = 2048;
						D_out[2] = 2048;

					}
					 flag_init=1;
				}else{}

				//Disturbance Observer
				for (i = 0; i < 3; i++){
					signal_dob1[i] = Gr[i][i] * Ktn[i][i] * i_a_ref[i] + g_dob[i][i] * Jn[i][i] * dth_res[i];
					signal_dob2[i] += g_dob[i][i] * (signal_dob1[i] - signal_dob2[i]) * sampling_time;
					hat_tau_dis[i] = signal_dob2[i] - g_dob[i][i] * Jn[i][i] * dth_res[i];
				}
				double a;
				a=0.15*(t-5);
				if(a>0.5){
					a=0.5;
				}
				th_cmd[0] = a;
				th_cmd[1] = a;
				th_cmd[2] = a;

				for (i = 0; i < 3; i++){
					ddth_ref[i] = Kpp_j[i][i] * (th_cmd[i] - th_res[i]) + Kpd_j[i][i] * (dth_cmd[i] - dth_res[i]);
				}

				for (i = 0; i < 3; i++){
					tau_ref[i] = Jn[i][i] * ddth_ref[i];
				}

				for  (i = 0; i < 3; i++){
					i_ref[i] = tau_ref[i] / (Gr[i][i] * Ktn[i][i]);
				}

				for  (i = 0; i < 3; i++){
					i_a_ref[i] = i_ref[i] + (hat_tau_dis[i] / (Gr[i][i] * Ktn[i][i]));
				}


				for (i = 0; i < 2; i++){
					D_out[i] = (i_a_ref[i] + 0.99) * 4096.0 / (0.396 * 5.0);
				}
				D_out[2] = (i_a_ref[2] + 1.5) * 4096.0 / (0.6 * 5.0);

			}
			if(t<20&&t>12){
//
				//位置制御コマンド
				x_w_cmd[0] = 0.4f;
				x_w_cmd[1] = 0.3f;
				x_w_cmd[2] = 0.0f;

				//力制御コマンド
				F_w_cmd[0] = 0.0f;
				F_w_cmd[1] = 0.0f;
				F_w_cmd[2] = 0.0f;

				//初期化
				if(flag_init==1){
					for(i=0;i<3;i++){
						i_a_ref[i] = 0.0f;
						 th_cmd[i] = 0.0f;
						 dth_cmd[i] = 0.0f;
						 ddth_ref[i] = 0.0f;
						 tau_ref[i] = 0.0f;
						 i_ref[i] = 0.0f;
						 signal_dob1[i] = 0.0f;
						 signal_dob2[i] = 0.0f;
						 hat_tau_dis[i] = 0.0f;
						 tau_cmd[i] = 0.0f;

					}
					 flag_init=2;
				}else{}
				J_aco[0][0]=-L1*sin(th_res[0])-L2*sin(th_res[0]+th_res[1])-L3*sin(th_res[0]+th_res[1]+th_res[2]);
				J_aco[0][1]=-L2*sin(th_res[0]+th_res[1])-L3*sin(th_res[0]+th_res[1]+th_res[2]);
				J_aco[0][2]=-L3*sin(th_res[0]+th_res[1]+th_res[2]);
				J_aco[1][0]=L1*cos(th_res[0])+L2*cos(th_res[0]+th_res[1])+L3*cos(th_res[0]+th_res[1]+th_res[2]);
				J_aco[1][1]=L2*cos(th_res[0]+th_res[1])+L3*cos(th_res[0]+th_res[1]+th_res[2]);
				J_aco[1][2]=L3*cos(th_res[0]+th_res[1]+th_res[2]);
				J_aco[2][0]=1.0;
				J_aco[2][1]=1.0;
				J_aco[2][2]=1.0;

				//Jaco^{-T}
				// 転置行列
				transpose_mat(3,3,J_aco,J_aco_T);

				inv_mat(3, J_aco_T, J_aco_T_inv);

				inv_mat(3, J_aco, J_aco_inv);

				for (i = 0; i < 3; i++){
					dx_w_res[i] = 0.0;
					for (j = 0; j < 3; j++){
						dx_w_res[i] += J_aco[i][j] * dth_res[j];
					}
				}

				x_w_res[0] = L2*cos(th_res[0] + th_res[1]) + L1*cos(th_res[0]) + L3*cos(th_res[0] + th_res[1] + th_res[2]);
				x_w_res[1] = L2*sin(th_res[0] + th_res[1]) + L1*sin(th_res[0]) + L3*sin(th_res[0] + th_res[1] + th_res[2]);
				x_w_res[2] = th_res[0] + th_res[1] + th_res[2];

				Mn[0][0] = I_z1 + 4.0*I_z2 + I_z3 + L1*L1*M2 + L1*L1*M3 + (L2*L2*M2)/4.0 + L2*L2*M3 + (L3*L3*M3)/4.0 + L1*L2*M2*cos(th_res[1]) + 2*L1*L2*M3*cos(th_res[1]) + L2*L3*M3*cos(th_res[2]) + L1*L3*M3*cos(th_res[1] + th_res[2]) + Jmn[0][0]*Gr[0][0]*Gr[0][0];
				Mn[1][1] = I_z3 + (L2*L2*M2)/4.0 + L2*L2*M3 + (L3*L3*M3)/4.0 + L2*L3*M3*cos(th_res[2])+Jmn[1][1]*Gr[1][1]*Gr[1][1];
				Mn[2][2] = (M3*L3*L3)/4 + I_z3+Jmn[2][2]*Gr[2][2]*Gr[2][2];

				//Disturbance Observer
				for (i = 0; i < 3; i++){
					signal_dob1[i] = Gr[i][i] * Ktn[i][i] * i_a_ref[i] + g_dob[i][i] * Mn[i][i] * dth_res[i];
					signal_dob2[i] += g_dob[i][i] * (signal_dob1[i] - signal_dob2[i]) * sampling_time;
					hat_tau_dis[i] = signal_dob2[i] - g_dob[i][i] * Mn[i][i] * dth_res[i];
				}

				//Reaction torque Observer
				for (i = 0; i < 3; i++){
					signal_rtob1[i] = Gr[i][i] * Ktn[i][i] * i_a_ref[i] + g_rtob[i][i] * Mn[i][i] * dth_res[i];
					signal_rtob2[i] += g_rtob[i][i] * (signal_dob1[i] - signal_rtob2[i]) * sampling_time;
					hat_tau_env[i] = signal_rtob2[i] - g_rtob[i][i] * Mn[i][i] * dth_res[i];
					hat_f_env[i]=0.0f;
					for (j = 0; j < 3; j++){
						hat_f_env[i] += J_aco_T_inv[i][j] * hat_tau_env[j];
					}

				}


				//Workspace Observer
				for (i = 0; i < 3; i++){
					signal_wob1[i] = tilde_ddx_w_ref[i] + g_wob[i][i] * dx_w_res[i];
					signal_wob2[i] += g_wob[i][i] * (signal_wob1[i] - signal_wob2[i]) * sampling_time;
					hat_ddx_w_dis[i] = signal_wob2[i] - g_wob[i][i] * dx_w_res[i];
				}


				for (i = 0; i < 3; i++){
					//位置のPD
					ddx_w_ref[i] = Kpp_w[i][i] * (x_w_cmd[i] - x_w_res[i]) + Kpd_w[i][i] * (dx_w_cmd[i] - dx_w_res[i]);
					//ddx_w_ref[i] = Kfp_w * (F_w_cmd[i] - hat_f_env[i]) + 0.5f * dx_w_res[i];
					//ddx_wp_ref[i] = Kpp_w[i][i] * (x_w_cmd[i] - x_w_res[i]) + Kpd_w[i][i] * (dx_w_cmd[i] - dx_w_res[i]);
					//ddx_wf_ref[i] = Kfp_w * (F_w_cmd[i] - hat_f_env[i]) + 0.5f * dx_w_res[i];
					//ddx_w_ref[i] = (I[i][i] - S[i][i]) * ddx_wp_ref[i] + S[i][i] * ddx_wf_ref[i];

				}

				for (i = 0; i < 3; i++){
					tilde_ddx_w_ref[i] = ddx_w_ref[i] + hat_ddx_w_dis[i];
				}


				for (i = 0; i < 3; i++){
					ddth_ref[i] = 0.0;
					for (j = 0; j < 3; j++){
						ddth_ref[i] += J_aco_inv[i][j] * tilde_ddx_w_ref[j];
					}
				}

				for (i = 0; i < 3; i++){
					tau_ref[i] = Mn[i][i] * ddth_ref[i];
				}

				for  (i = 0; i < 3; i++){
					i_ref[i] = tau_ref[i] / (Gr[i][i] * Ktn[i][i]);
				}

				for  (i = 0; i < 3; i++){
					i_a_ref[i] = i_ref[i] + (hat_tau_dis[i] / (Gr[i][i] * Ktn[i][i]));
				}


				for (i = 0; i < 2; i++){
					D_out[i] = (i_a_ref[i] + 0.99) * 4096.0 / (0.396 * 5.0);
				}
				D_out[2] = (i_a_ref[2] + 1.5) * 4096.0 / (0.6 * 5.0);


				//hybrid
			}else if(t>25){
				//位置制御コマンド
				double b;
				b=-0.05*(t-20);
				if(b<0.0){
					b=0.0;
				}
				x_w_cmd[0] = 0.0f;
				x_w_cmd[1] = b;
				x_w_cmd[2] = 0.0f;


				//力制御コマンド
				F_w_cmd[0] = 0.00f;
				F_w_cmd[1] = 0.0f;
				F_w_cmd[2] = 0.0f;

				//初期化
				if(flag_init==2){
					for(i=0;i<3;i++){
						i_a_ref[i] = 0.0f;
						 th_cmd[i] = 0.0f;
						 dth_cmd[i] = 0.0f;
						 ddth_ref[i] = 0.0f;
						 tau_ref[i] = 0.0f;
						 i_ref[i] = 0.0f;
						 signal_dob1[i] = 0.0f;
						 signal_dob2[i] = 0.0f;
						 hat_tau_dis[i] = 0.0f;
						 tau_cmd[i] = 0.0f;

					}
					 flag_init=3;
				}else{}
				J_aco[0][0]=-L1*sin(th_res[0])-L2*sin(th_res[0]+th_res[1])-L3*sin(th_res[0]+th_res[1]+th_res[2]);
				J_aco[0][1]=-L2*sin(th_res[0]+th_res[1])-L3*sin(th_res[0]+th_res[1]+th_res[2]);
				J_aco[0][2]=-L3*sin(th_res[0]+th_res[1]+th_res[2]);
				J_aco[1][0]=L1*cos(th_res[0])+L2*cos(th_res[0]+th_res[1])+L3*cos(th_res[0]+th_res[1]+th_res[2]);
				J_aco[1][1]=L2*cos(th_res[0]+th_res[1])+L3*cos(th_res[0]+th_res[1]+th_res[2]);
				J_aco[1][2]=L3*cos(th_res[0]+th_res[1]+th_res[2]);
				J_aco[2][0]=1.0;
				J_aco[2][1]=1.0;
				J_aco[2][2]=1.0;

				//Jaco^{-T}

				// 転置行列
				transpose_mat(3,3,J_aco,J_aco_T);

				inv_mat(3, J_aco_T, J_aco_T_inv);

				inv_mat(3, J_aco, J_aco_inv);

				//等価質量行列(J_aco_T_inv*Jn*J_aco_inv)

				mul_mat(3,3,3,3,J_aco_T_inv,Jn,J_aco_Jn);

				mul_mat(3,3,3,3,J_aco_Jn,J_aco_inv,M_w_eq);

				//printf("%lf \n\r",M_w_eq[0][0]);



				for (i = 0; i < 3; i++){
					dx_w_res[i] = 0.0;
					for (j = 0; j < 3; j++){
						dx_w_res[i] += J_aco[i][j] * dth_res[j];
					}
				}

				x_w_res[0] = L2*cos(th_res[0] + th_res[1]) + L1*cos(th_res[0]) + L3*cos(th_res[0] + th_res[1] + th_res[2]);
				x_w_res[1] = L2*sin(th_res[0] + th_res[1]) + L1*sin(th_res[0]) + L3*sin(th_res[0] + th_res[1] + th_res[2]);
				x_w_res[2] = th_res[0] + th_res[1] + th_res[2];

				Mn[0][0] = I_z1 + 4.0*I_z2 + I_z3 + L1*L1*M2 + L1*L1*M3 + (L2*L2*M2)/4.0 + L2*L2*M3 + (L3*L3*M3)/4.0 + L1*L2*M2*cos(th_res[1]) + 2*L1*L2*M3*cos(th_res[1]) + L2*L3*M3*cos(th_res[2]) + L1*L3*M3*cos(th_res[1] + th_res[2]) + Jmn[0][0]*Gr[0][0]*Gr[0][0];
				Mn[1][1] = I_z3 + (L2*L2*M2)/4.0 + L2*L2*M3 + (L3*L3*M3)/4.0 + L2*L3*M3*cos(th_res[2])+Jmn[1][1]*Gr[1][1]*Gr[1][1];
				Mn[2][2] = (M3*L3*L3)/4 + I_z3+Jmn[2][2]*Gr[2][2]*Gr[2][2];

				//Disturbance Observer
				for (i = 0; i < 3; i++){
					signal_dob1[i] = Gr[i][i] * Ktn[i][i] * i_a_ref[i] + g_dob[i][i] * Mn[i][i] * dth_res[i];
					signal_dob2[i] += g_dob[i][i] * (signal_dob1[i] - signal_dob2[i]) * sampling_time;
					hat_tau_dis[i] = signal_dob2[i] - g_dob[i][i] * Mn[i][i] * dth_res[i];
				}

				//Reaction torque Observer
				for (i = 0; i < 3; i++){
					signal_rtob1[i] = Gr[i][i] * Ktn[i][i] * i_a_ref[i] + g_rtob[i][i] * Mn[i][i] * dth_res[i];
					signal_rtob2[i] += g_rtob[i][i] * (signal_dob1[i] - signal_rtob2[i]) * sampling_time;
					hat_tau_env[i] = signal_rtob2[i] - g_rtob[i][i] * Mn[i][i] * dth_res[i];
					hat_f_env[i]=0.0f;
					for (j = 0; j < 3; j++){
						hat_f_env[i] += J_aco_T_inv[i][j] * hat_tau_env[j];
					}

				}


				//Workspace Observer
				for (i = 0; i < 3; i++){
					signal_wob1[i] = tilde_ddx_w_ref[i] + g_wob[i][i] * dx_w_res[i];
					signal_wob2[i] += g_wob[i][i] * (signal_wob1[i] - signal_wob2[i]) * sampling_time;
					hat_ddx_w_dis[i] = signal_wob2[i] - g_wob[i][i] * dx_w_res[i];
				}


				/* MPCによる指令値生成：X方向のEKF推定値と等価質量を使用する。 */
                const double equivalent_mass = (double)M_w_eq[0][0];
                mpc_sensor_status = 0;
                if (!isfinite(equivalent_mass) || equivalent_mass <= 0.0)
                {
                    /* 無効な質量をEKF/MPCへ渡さず、前回のMPC結果も無効化する。 */
                    mpc_mass_update_status = MPC_UpdateModelMass(equivalent_mass);
                    mpc_update_status = mpc_mass_update_status;
                    mpc_step_status = mpc_mass_update_status;
                    mpc_sensor_status = -1;
                }
                else
                {
                    /* 既存EKFを呼ぶ。観測をX方向の速度・位置にそろえる。 */
                    y_k[0] = dx_w_res[0];
                    y_k[1] = x_w_res[0];
                    EKF(y_k, u_k_1, equivalent_mass, x_hat_k_1,
                        P_k_1, Rkf, Qkf, x_hat_k, P_k);

                    /* 状態4個すべてを今回のEKF推定値から用意する。 */
                    const double measured_state[4] = {
                        (double)x_hat_k[0], (double)x_hat_k[1],
                        (double)x_hat_k[2], (double)x_hat_k[3]
                    };
                    if (hat_f_env[0] > 0)
                    {
                        /* 既存の接触時だけ計算する条件を維持する。 */
                        mpc_sensor_status = 1;
                        const uint32_t start_cycles = mpc_dwt_status == 0
                                                    ? MPC_CycleCounterRead() : 0U;
                        mpc_mass_update_status = MPC_UpdateModelMass(equivalent_mass);
                        mpc_update_status = -999;
                        mpc_step_status = mpc_mass_update_status;
                        if (mpc_mass_update_status == 0)
                        {
                            mpc_sample_count++;
                            mpc_update_status = MPC_UpdateInitialState(measured_state);
                            mpc_step_status = mpc_update_status;
                            if (mpc_update_status == 0)
                                mpc_step_status = MPC_Execute();
                        }
                        /* EKFより後のMPC更新～結果取得をDWTで計測する。 */
                        if (mpc_dwt_status == 0)
                        {
                            mpc_total_cycles = MPC_CycleCounterRead() - start_cycles;
                            mpc_total_time_us = MPC_CyclesToMicroseconds(mpc_total_cycles);
                            if (mpc_total_time_us > mpc_max_time_us)
                                mpc_max_time_us = mpc_total_time_us;
                            if ((uint64_t)mpc_total_cycles * 1000ULL > SystemCoreClock)
                                mpc_overrun_count++;
                        }
                    }
                }


				for (i = 0; i < 3; i++){
					//位置のPD
					//ddx_w_ref[i] = Kpp_w[i][i] * (x_w_cmd[i] - x_w_res[i]) + Kpd_w[i][i] * (dx_w_cmd[i] - dx_w_res[i]);
					//ddx_w_ref[i] = Kfp_w * (F_w_cmd[i] - hat_f_env[i]) + 0.5f * dx_w_res[i];
					ddx_wp_ref[i] = Kpp_w[i][i] * (x_w_cmd[i] - x_w_res[i]) + Kpd_w[i][i] * (dx_w_cmd[i] - dx_w_res[i]);
					ddx_wf_ref[i] = Kfp_w * (F_w_cmd[i] - hat_f_env[i]) + 5.0f * dx_w_res[i];
					ddx_w_ref[i] = (I[i][i] - S[i][i]) * ddx_wp_ref[i] + S[i][i] * ddx_wf_ref[i];

				}
				/* 今回のMPC計算が成功した場合、Y方向の指令を更新する。
				 * mpc_u0の物理量・単位が加速度指令に一致することが前提。
				 */
				if (mpc_sensor_status == 1 &&
                    mpc_mass_update_status == 0 &&
				    mpc_update_status == 0 &&
				    mpc_step_status == 0 &&
				    mpc_result_valid)
				{
					/*加速度指令値に変換*/
				    ddx_w_ref[0] = (float)(mpc_u0 / mpc_model_mass);
				}

				for (i = 0; i < 3; i++){
					tilde_ddx_w_ref[i] = ddx_w_ref[i] + hat_ddx_w_dis[i];
				}


				for (i = 0; i < 3; i++){
					ddth_ref[i] = 0.0;
					for (j = 0; j < 3; j++){
						ddth_ref[i] += J_aco_inv[i][j] * tilde_ddx_w_ref[j];
					}
				}

				for (i = 0; i < 3; i++){
					tau_ref[i] = Mn[i][i] * ddth_ref[i];
				}

				for  (i = 0; i < 3; i++){
					i_ref[i] = tau_ref[i] / (Gr[i][i] * Ktn[i][i]);
				}

				for  (i = 0; i < 3; i++){
					i_a_ref[i] = i_ref[i] + (hat_tau_dis[i] / (Gr[i][i] * Ktn[i][i]));
				}


				for (i = 0; i < 2; i++){
					D_out[i] = (i_a_ref[i] + 0.99) * 4096.0 / (0.396 * 5.0);
				}
				D_out[2] = (i_a_ref[2] + 1.5) * 4096.0 / (0.6 * 5.0);

			}else{}


			for (i = 0; i < 3; i++){
				if (D_out[i] > 4095) {
					D_out[i] = 4095;
				}
				else if (D_out[i] < 0) {
					D_out[i] = 0;
				}
				else{}
			}


//			D_out[0] = 2048;
//			D_out[1] = 2048;
//			D_out[2] = 2048;

			MCP4922_Write_DAC1(0, D_out[0]);
			MCP4922_Write_DAC1(1, D_out[1]);
			MCP4922_Write_DAC2(0, D_out[2]);

			//EKFの推定値と共分散行列の更新
			copy_vec(4, x_hat_k, x_hat_k_1);
			copy_mat(4, 4, P_k, P_k_1);
			u_k_1[0]=hat_f_env[0];
			if(t < t_end && cnt_time % 10 == 0){
				//printf("%lf, %lf, %lf, %lf, %lf, %lf, %lf,%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf \n\r", t, th_cmd[0], th_cmd[1], th_cmd[2], th_res[0], th_res[1], th_res[2],x_w_cmd[0],x_w_cmd[1],x_w_cmd[2],x_w_res[0],x_w_res[1],x_w_res[2],F_w_cmd[0],hat_f_env[0]);
				///printf("%lf, %lf, %lf, %lf, %lf, %lf, %lf,% lf, %lf,  \n\r", t, th_res[0], th_res[1], th_res[2], x_w_res[0], x_w_res[1], x_w_res[2], F_w_cmd[0],hat_f_env[0]);
				//printf("%ld, %ld, %ld\n\r", cnt_encoder[0], cnt_encoder[1], cnt_encoder[2]);
				//printf("%ld, %ld, %ld\n\r", D_out[0],D_out[1],D_out[2]);
				//printf("%lf, %lf, %lf, %lf\n\r", t,i_a_ref[0],i_a_ref[1],i_a_ref[2]);
				//printf("%lf, %lf, %lf, %lf\n\r", t, th_res[0], th_res[1], th_res[2]);
			}else{}
			t += sampling_time;
			cnt_time ++;
		}else{
			HAL_GPIO_WritePin(On_Servo_GPIO_Port, On_Servo_Pin, GPIO_PIN_RESET);
		}

	}else{}

	/* Encoder read of Joint 1 */
	if (htim->Instance == TIM3){
		__HAL_TIM_CLEAR_FLAG(&htim3, TIM_IT_UPDATE);
		if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3)){
			overflowcnt[0] --;
		}
		else {
			overflowcnt[0] ++;
		}
	}else{}

	/* Encoder read of Joint 2 */
	if (htim->Instance == TIM4){
		__HAL_TIM_CLEAR_FLAG(&htim4, TIM_IT_UPDATE);
		if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim4)){
			overflowcnt[1] --;
		}
		else {
			overflowcnt[1] ++;
		}
	}else{}

	/* Encoder read of Joint 3 */
	if (htim->Instance == TIM1){
		__HAL_TIM_CLEAR_FLAG(&htim1, TIM_IT_UPDATE);
		if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim1)){
			overflowcnt[2] --;
		}
		else {
			overflowcnt[2] ++;
		}
	}else{}

}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
      dma_printf_send_it(huart);
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
