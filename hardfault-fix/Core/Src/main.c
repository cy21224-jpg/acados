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
/* Absolute tolerance in each variable's model units; not a safety margin. */
#define MPC_BOUND_TOL 1.0e-6
#define MPC_CHECK_SOLVER_FAILED  1U
#define MPC_CHECK_NONFINITE      2U
#define MPC_CHECK_INPUT_BOUNDS   4U
#define MPC_CHECK_STATE_BOUNDS   8U
#define MPC_CHECK_MODEL_CONFIG 16U

/* This diagnostic targets the supplied one-input, four-state model. */
_Static_assert(INTERACTION_MODEL_NU == 1 && INTERACTION_MODEL_NBU == 1,
               "Update diagnostics for the new control dimensions");
_Static_assert(INTERACTION_MODEL_NBX0 == INTERACTION_MODEL_NX &&
               INTERACTION_MODEL_NBX == INTERACTION_MODEL_NX &&
               INTERACTION_MODEL_NBXN == INTERACTION_MODEL_NX,
               "Update diagnostics for the new state bound dimensions");

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

static interaction_model_solver_capsule *mpc_capsule = NULL;

volatile int mpc_create_status = -999;
volatile int mpc_solve_status = -999;
volatile uint32_t g_heap_peak_bytes = 0U;

/* Expressions: -999 = not checked; 0 = numeric/bound checks passed.
   Nonzero values are a bit mask of MPC_CHECK_* above. */
volatile int mpc_check_status = -999;
volatile int mpc_check_done = 0;
volatile int mpc_first_bad_stage = -1;
volatile double mpc_u0 = NAN;
volatile double mpc_u_lower[INTERACTION_MODEL_N];
volatile double mpc_u_upper[INTERACTION_MODEL_N];
volatile double mpc_u_pred[INTERACTION_MODEL_N];
volatile double mpc_x_pred[INTERACTION_MODEL_N + 1][INTERACTION_MODEL_NX];
volatile double mpc_max_u_violation = 0.0;
volatile double mpc_max_x_violation = 0.0;
/* A range at x[0] is not an actual measured initial state. */
volatile int mpc_initial_state_fixed = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */

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


/* Read the actual solver bounds rather than duplicating generated constants.
   This is a diagnostic only: it does not command an actuator or clamp output. */
static void MPC_CheckSolution(void)
{
    unsigned int flags = 0U;
    int first_bad = -1;
    int initial_fixed = 1;
    double max_u_violation = 0.0;
    double max_x_violation = 0.0;
    mpc_check_done = 0;
    mpc_check_status = -999;
    mpc_first_bad_stage = -1;
    mpc_u0 = NAN;
    mpc_initial_state_fixed = 0;
    mpc_max_u_violation = 0.0;
    mpc_max_x_violation = 0.0;
    for (int k = 0; k <= INTERACTION_MODEL_N; k++)
    {
        for (int j = 0; j < INTERACTION_MODEL_NX; j++)
            mpc_x_pred[k][j] = NAN;
        if (k < INTERACTION_MODEL_N)
        {
            mpc_u_pred[k] = NAN;
            mpc_u_lower[k] = NAN;
            mpc_u_upper[k] = NAN;
        }
    }
    if (mpc_solve_status != 0)
    {
        /* Do not present an unsuccessful solve's iterate as a valid command. */
        mpc_check_status = MPC_CHECK_SOLVER_FAILED;
        mpc_check_done = 1;
        return;
    }
    if (mpc_capsule == NULL || mpc_capsule->nlp_solver_plan->N != INTERACTION_MODEL_N)
    {
        mpc_check_status = MPC_CHECK_MODEL_CONFIG;
        mpc_check_done = 1;
        return;
    }
    for (int k = 0; k <= INTERACTION_MODEL_N; k++)
    {
        unsigned int stage_flags = 0U;
        double x[INTERACTION_MODEL_NX];
        double lbx[INTERACTION_MODEL_NX];
        double ubx[INTERACTION_MODEL_NX];
        int idxbx[INTERACTION_MODEL_NX];
        ocp_nlp_out_get(mpc_capsule->nlp_config, mpc_capsule->nlp_dims,
                        mpc_capsule->nlp_out, k, "x", x);
        ocp_nlp_constraints_model_get(mpc_capsule->nlp_config, mpc_capsule->nlp_dims,
                        mpc_capsule->nlp_in, k, "lbx", lbx);
        ocp_nlp_constraints_model_get(mpc_capsule->nlp_config, mpc_capsule->nlp_dims,
                        mpc_capsule->nlp_in, k, "ubx", ubx);
        ocp_nlp_constraints_model_get(mpc_capsule->nlp_config, mpc_capsule->nlp_dims,
                        mpc_capsule->nlp_in, k, "idxbx", idxbx);
        for (int j = 0; j < INTERACTION_MODEL_NX; j++)
        {
            mpc_x_pred[k][j] = x[j];
            if (!isfinite(x[j])) stage_flags |= MPC_CHECK_NONFINITE;
            if (idxbx[j] < 0 || idxbx[j] >= INTERACTION_MODEL_NX ||
                !isfinite(lbx[j]) || !isfinite(ubx[j]) || lbx[j] > ubx[j])
            {
                stage_flags |= MPC_CHECK_MODEL_CONFIG;
                if (k == 0) initial_fixed = 0;
                continue;
            }
            if (k == 0 && lbx[j] != ubx[j]) initial_fixed = 0;
            if (isfinite(x[idxbx[j]]))
            {
                double v = fmax(0.0, fmax(lbx[j] - x[idxbx[j]], x[idxbx[j]] - ubx[j]));
                if (v > max_x_violation) max_x_violation = v;
                if (v > MPC_BOUND_TOL) stage_flags |= MPC_CHECK_STATE_BOUNDS;
            }
        }
        if (k < INTERACTION_MODEL_N)
        {
            double u[INTERACTION_MODEL_NU], lbu[INTERACTION_MODEL_NBU], ubu[INTERACTION_MODEL_NBU];
            int idxbu[INTERACTION_MODEL_NBU];
            ocp_nlp_out_get(mpc_capsule->nlp_config, mpc_capsule->nlp_dims,
                            mpc_capsule->nlp_out, k, "u", u);
            ocp_nlp_constraints_model_get(mpc_capsule->nlp_config, mpc_capsule->nlp_dims,
                            mpc_capsule->nlp_in, k, "lbu", lbu);
            ocp_nlp_constraints_model_get(mpc_capsule->nlp_config, mpc_capsule->nlp_dims,
                            mpc_capsule->nlp_in, k, "ubu", ubu);
            ocp_nlp_constraints_model_get(mpc_capsule->nlp_config, mpc_capsule->nlp_dims,
                            mpc_capsule->nlp_in, k, "idxbu", idxbu);
            mpc_u_pred[k] = u[0];
            mpc_u_lower[k] = lbu[0];
            mpc_u_upper[k] = ubu[0];
            if (k == 0) mpc_u0 = u[0];
            if (!isfinite(u[0])) stage_flags |= MPC_CHECK_NONFINITE;
            if (idxbu[0] != 0 || !isfinite(lbu[0]) || !isfinite(ubu[0]) || lbu[0] > ubu[0])
                stage_flags |= MPC_CHECK_MODEL_CONFIG;
            else if (isfinite(u[0]))
            {
                double v = fmax(0.0, fmax(lbu[0] - u[0], u[0] - ubu[0]));
                if (v > max_u_violation) max_u_violation = v;
                if (v > MPC_BOUND_TOL) stage_flags |= MPC_CHECK_INPUT_BOUNDS;
            }
        }
        if (stage_flags != 0U && first_bad < 0) first_bad = k;
        flags |= stage_flags;
    }
    mpc_initial_state_fixed = initial_fixed;
    mpc_max_u_violation = max_u_violation;
    mpc_max_x_violation = max_x_violation;
    mpc_first_bad_stage = first_bad;
    mpc_check_status = (int)flags;
    mpc_check_done = 1;
}

static void MPC_PrintDiagnostics(void)
{
    printf("\r\nMPC solve=%d check=%d (%s) first_bad_stage=%d\r\n",
           (int)mpc_solve_status, (int)mpc_check_status,
           mpc_check_status == 0 ? "NUMERIC_BOUNDS_PASS" : "FAIL",
           (int)mpc_first_bad_stage);
    printf("flags: solver=1 nonfinite=2 input_bounds=4 state_bounds=8 config=16\r\n");
    if (mpc_solve_status == 0 && (mpc_check_status & MPC_CHECK_MODEL_CONFIG) == 0)
    {
        printf("u0=%.9f bounds=[%.9f, %.9f] max_u_violation=%.3e max_x_violation=%.3e\r\n",
               (double)mpc_u0, (double)mpc_u_lower[0], (double)mpc_u_upper[0],
               (double)mpc_max_u_violation, (double)mpc_max_x_violation);
        for (int k = 0; k < INTERACTION_MODEL_N; k++)
            printf("u[%d]=%.9f bounds=[%.9f, %.9f]\r\n", k,
                   (double)mpc_u_pred[k], (double)mpc_u_lower[k], (double)mpc_u_upper[k]);
        for (int k = 0; k <= INTERACTION_MODEL_N; k++)
            printf("x[%d]=[%.9f, %.9f, %.9f, %.9f]\r\n", k,
                   (double)mpc_x_pred[k][0], (double)mpc_x_pred[k][1],
                   (double)mpc_x_pred[k][2], (double)mpc_x_pred[k][3]);
        if (!mpc_initial_state_fixed)
            printf("WARNING: x0 is a range, not fixed to a measured state.\r\n");
    }
    printf("Numeric/bound checks only; physical control suitability is not verified.\r\n");
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
  /* 1. Create acados solver capsule */
  mpc_capsule = interaction_model_acados_create_capsule();

  if (mpc_capsule == NULL)
  {
      mpc_create_status = -1;
      Error_Handler();
  }

  /* 2. Create MPC solver */
  mpc_create_status =
      interaction_model_acados_create(mpc_capsule);

  if (mpc_create_status != 0)
  {
      Error_Handler();
  }

  /* MPC最適化計算を1回実行 */
  mpc_solve_status =
      interaction_model_acados_solve(mpc_capsule);

  MPC_CheckSolution();
  MPC_PrintDiagnostics();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  /* Diagnostics are printed once after solve, without flooding UART. */
      HAL_Delay(1000);
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
