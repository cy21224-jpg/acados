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
#include <stdio.h>
#include <math.h>
#include "acados_solver_interaction_model.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 計算終了後の待ち時間。計算・UART出力の時間も加わるため、厳密な1 ms周期ではない。 */
#define MPC_LOOP_DELAY_MS 1U
/* UARTの連続出力で計算を妨げすぎないよう、通常は1秒ごとに表示する。 */
#define MPC_REPORT_INTERVAL_MS 1000U
/* このコードは入力1個、状態4個、予測区間2個以上の現在のモデルを対象とする。 */
_Static_assert(INTERACTION_MODEL_NU == 1 && INTERACTION_MODEL_NX == 4 &&
               INTERACTION_MODEL_N >= 2, "Update result buffers for the new model");

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

TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

/* ソルバ本体とアクセス用ポインタを保持し、while内で再生成しない。 */
static interaction_model_solver_capsule *mpc_capsule = NULL;
static ocp_nlp_config *mpc_config = NULL;
static ocp_nlp_dims *mpc_dims = NULL;
static ocp_nlp_out *mpc_out = NULL;
static uint32_t mpc_last_report_tick = 0U;

/* 初期化と直近の計算の戻り値。-999は未実行を表す。 */
volatile int mpc_create_status = -999;
volatile int mpc_solve_status = -999;
volatile uint32_t g_heap_peak_bytes = 0U;
/* デバッガのExpressionsで計算回数・失敗回数・最新出力の有効性を確認する。 */
volatile uint32_t mpc_iteration_count = 0U;
volatile uint32_t mpc_success_count = 0U;
volatile uint32_t mpc_failure_count = 0U;
volatile int mpc_result_valid = 0;
/* HAL_GetTickによる粗い計算時間。1 ms未満は0になることがある。 */
volatile uint32_t mpc_solve_time_ms = 0U;
/* u0は今回使う最初の入力、u1は次の予測ステップの入力。別々の入力軸ではない。 */
volatile double mpc_u0 = NAN;
volatile double mpc_u1 = NAN;
/* ソルバのstage 0の状態。センサから読み取った実測状態ではない。 */
volatile double mpc_x0[INTERACTION_MODEL_NX] = {NAN, NAN, NAN, NAN};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */
/* 初期化・1回分の計算・UART表示を分け、実行する場所を明確にする。 */
static int MPC_Init(void);
static void MPC_RunOnce(void);
static void MPC_PrintResult(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF);
	return ch;
}


/* ソルバのメモリ確保・モデル初期化・ポインタ取得を起動時に一度だけ行う。 */
static int MPC_Init(void)
{
    /* 誤って複数回呼ばれても、カプセルを再生成してメモリを失わない。 */
    if (mpc_capsule != NULL)
        return mpc_create_status;

    /* カプセルを確保し、確保できなければ呼び出し元に失敗を返す。 */
    mpc_capsule = interaction_model_acados_create_capsule();
    if (mpc_capsule == NULL)
    {
        mpc_create_status = -1;
        return mpc_create_status;
    }

    /* acadosのモデル、制約、計算用メモリを一度だけ構築する。 */
    mpc_create_status = interaction_model_acados_create(mpc_capsule);
    if (mpc_create_status != 0)
        return mpc_create_status;

    /* ループで結果を取得するときに使うポインタを保存する。 */
    mpc_config = interaction_model_acados_get_nlp_config(mpc_capsule);
    mpc_dims = interaction_model_acados_get_nlp_dims(mpc_capsule);
    mpc_out = interaction_model_acados_get_nlp_out(mpc_capsule);
    if (mpc_config == NULL || mpc_dims == NULL || mpc_out == NULL)
    {
        mpc_create_status = -2;
        return mpc_create_status;
    }
    return 0;
}

/* 確保済みのソルバを再利用し、現在の設定から制御入力を1回計算する。 */
static void MPC_RunOnce(void)
{
    /* 前回の入力を今回の成功結果と取り違えないよう、計算前に無効化する。 */
    mpc_result_valid = 0;
    mpc_u0 = NAN;
    mpc_u1 = NAN;
    for (int i = 0; i < INTERACTION_MODEL_NX; i++)
        mpc_x0[i] = NAN;

    /* 初期化成功を確認してから、同じカプセルでsolveを呼ぶ。 */
    if (mpc_create_status != 0 || mpc_capsule == NULL)
    {
        mpc_solve_status = -998;
        return;
    }
    uint32_t start_tick = HAL_GetTick();
    mpc_solve_status = interaction_model_acados_solve(mpc_capsule);
    mpc_solve_time_ms = HAL_GetTick() - start_tick;
    mpc_iteration_count++;

    /* 計算失敗時は古い結果を再利用せず、失敗回数を記録して次のループへ戻る。 */
    if (mpc_solve_status != 0)
    {
        mpc_failure_count++;
        return;
    }

    /* stage 0とstage 1の入力、およびstage 0の状態をローカル変数に取得する。 */
    double u0[INTERACTION_MODEL_NU] = {0};
    double u1[INTERACTION_MODEL_NU] = {0};
    double x0[INTERACTION_MODEL_NX] = {0};
    ocp_nlp_out_get(mpc_config, mpc_dims, mpc_out, 0, "u", u0);
    ocp_nlp_out_get(mpc_config, mpc_dims, mpc_out, 1, "u", u1);
    ocp_nlp_out_get(mpc_config, mpc_dims, mpc_out, 0, "x", x0);

    /* 戻り値が0でもNaN/Infが含まれる場合は、有効な出力として公開しない。 */
    int finite_result = isfinite(u0[0]) && isfinite(u1[0]);
    for (int i = 0; i < INTERACTION_MODEL_NX; i++)
        finite_result = finite_result && isfinite(x0[i]);
    if (!finite_result)
    {
        mpc_failure_count++;
        return;
    }

    /* 検査を通った値をExpressionsと後段の制御処理で参照できる変数に保存する。 */
    mpc_u0 = u0[0];
    mpc_u1 = u1[0];
    for (int i = 0; i < INTERACTION_MODEL_NX; i++)
        mpc_x0[i] = x0[i];
    mpc_success_count++;
    mpc_result_valid = 1;
}

/* 最新の計算状態と取得した入力・状態をUARTへ表示する。表示中は計算が止まる。 */
static void MPC_PrintResult(void)
{
    printf("\r\n========== MPC Result ==========\r\n");
    printf("iteration=%lu success=%lu failure=%lu\r\n",
           (unsigned long)mpc_iteration_count, (unsigned long)mpc_success_count,
           (unsigned long)mpc_failure_count);
    printf("MPC solve status = %d, result_valid = %d, solve_ms = %lu\r\n",
           (int)mpc_solve_status, (int)mpc_result_valid, (unsigned long)mpc_solve_time_ms);
    if (mpc_result_valid)
    {
        printf("mpc_u0 = %.10f\r\n", (double)mpc_u0);
        printf("mpc_u1 = %.10f\r\n", (double)mpc_u1);
        for (int i = 0; i < INTERACTION_MODEL_NX; i++)
            printf("mpc_x0[%d] = %.10f\r\n", i, (double)mpc_x0[i]);
    }
    else
        printf("No valid result: previous outputs were invalidated.\r\n");
    printf("================================\r\n");
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
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  /* 起動時の初期化：ソルバを一度だけ作成し、失敗したら計算ループに入らない。 */
  if (MPC_Init() != 0)
  {
      printf("Error: MPC initialization failed: %d\r\n", (int)mpc_create_status);
      Error_Handler();
  }
  /* 通常の結果表示を1秒ごとに制限するため、表示時刻を初期化する。 */
  mpc_last_report_tick = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      /* 1. 実測状態を使う場合は、ここでセンサを読み、stage 0のlbx/ubxを更新する。
         現在はセンサ仕様が未定なので、生成済みモデルの初期状態範囲を維持する。
         ソルバが求めたmpc_x0を実測状態と見なして戻してはいけない。 */

      /* 2. 起動時に作成したソルバで毎ループ計算し、mpc_u0/mpc_u1/mpc_x0を更新する。 */
      MPC_RunOnce();

      /* 3. 実機に入力する処理を追加する場合は、mpc_result_validを確認してからu0を使う。
         今回はアクチュエータへの出力処理は行わない。 */

      /* 4. 最初の結果と、その後は1秒ごとの結果をUARTへ表示する。 */
      if (mpc_iteration_count == 1U ||
          (uint32_t)(HAL_GetTick() - mpc_last_report_tick) >= MPC_REPORT_INTERVAL_MS)
      {
          MPC_PrintResult();
          mpc_last_report_tick = HAL_GetTick();
      }

      /* 5. 動作確認用に計算後1 ms待つ。実際の周期は計算・表示時間の分だけ長くなる。 */
      HAL_Delay(MPC_LOOP_DELAY_MS);
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
  huart3.Init.BaudRate = 115200;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

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
