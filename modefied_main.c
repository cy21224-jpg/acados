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
/* 現在のモデルの入力制約。初期化後はMPC_UpdateInputBounds()でも変更できる。 */
#define MPC_INPUT_MIN 0.0
#define MPC_INPUT_MAX 10.0
/* センサ4状態と、1入力・2段以上の予測入力を扱う現在のモデルに限定する。 */
_Static_assert(INTERACTION_MODEL_NX == 4 && INTERACTION_MODEL_NU == 1 &&
               INTERACTION_MODEL_NBX0 == 4 && INTERACTION_MODEL_NBU == 1 &&
               INTERACTION_MODEL_N >= 2, "Update sensor interface for new model dimensions");
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

/* 起動時に作成したソルバと入出力を保持し、毎回のメモリ確保を避ける。 */
static interaction_model_solver_capsule *mpc_capsule = NULL;
static ocp_nlp_config *mpc_config = NULL;
static ocp_nlp_dims *mpc_dims = NULL;
static ocp_nlp_in *mpc_in = NULL;
static ocp_nlp_out *mpc_out = NULL;
static int mpc_state_pending = 0;
/* Linux版と同様、前回成功時の入力列を次回の初期推定に使う。 */
static int mpc_has_warm_start = 0;
static double mpc_previous_u[INTERACTION_MODEL_N];
static uint32_t mpc_last_report_tick = 0U;

/* センサ側は4値を同じサンプルとして書き、最後にready=1を設定する。
   デバッガで試す場合も4値を書いてからsensor_sample_ready=1にする。
   データを読んだ後はready=0に戻る。実センサの取得処理そのものは未実装。 */
volatile double sensor_state[4] = {0.0, 0.0, 0.0, 0.0};
volatile int sensor_sample_ready = 0;

/* -999は未実行。センサ読取りは0=未着、1=取得、-1=非有限値。 */
volatile int mpc_create_status = -999;
volatile int mpc_sensor_status = 0;
volatile int mpc_update_status = -999;
volatile int mpc_step_status = -999;
volatile int mpc_solve_status = -999;
volatile uint32_t g_heap_peak_bytes = 0U;
/* sample_countは取得したデータ数、iteration_countは実際のsolve回数。 */
volatile uint32_t mpc_sample_count = 0U;
volatile uint32_t mpc_iteration_count = 0U;
volatile uint32_t mpc_success_count = 0U;
volatile uint32_t mpc_failure_count = 0U;
/* valid=1は直近の成功結果。新しいサンプルが来たという意味ではない。 */
volatile int mpc_result_valid = 0;
volatile uint32_t mpc_result_sample_id = 0U;
volatile uint32_t mpc_solve_time_ms = 0U;
volatile double mpc_input_lower = MPC_INPUT_MIN;
volatile double mpc_input_upper = MPC_INPUT_MAX;
/* measured_xは実測入力、x0はソルバ出力。両者を混同しない。 */
volatile double mpc_measured_x[4] = {NAN, NAN, NAN, NAN};
volatile double mpc_x0[4] = {NAN, NAN, NAN, NAN};
volatile double mpc_u0 = NAN;
volatile double mpc_u1 = NAN;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */
/* センサ取得、初期化、制約更新、計算実行、表示の役割を関数で分離する。 */
static int Sensors_ReadState(double state[4]);
static int MPC_Init(void);
static void MPC_InvalidateResult(void);
static int MPC_UpdateInitialState(const double measured_state[4]);
static int MPC_UpdateInputBounds(double lower, double upper);
static void MPC_PrepareWarmStart(void);
static int MPC_Execute(void);
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


/* センサが用意した4状態を、割込みによる書き換えから保護してコピーする。
   センサ側の同一割込みで4値を書き、最後にready=1とする運用を想定する。
   main側のコードを置き換える場合は、この関数内で実センサを読んでもよい。 */
static int Sensors_ReadState(double state[4])
{
    /* 短いコピーの間だけ割込みを止め、元の割込み許可状態に必ず戻す。 */
    uint32_t saved_primask = __get_PRIMASK();
    __disable_irq();
    if (sensor_sample_ready == 0)
    {
        __set_PRIMASK(saved_primask);
        return 0;
    }
    for (int i = 0; i < 4; i++)
        state[i] = sensor_state[i];
    sensor_sample_ready = 0;
    __set_PRIMASK(saved_primask);

    /* NaN/Infのサンプルはモデルに渡さず、呼び出し元へ失敗を返す。 */
    for (int i = 0; i < 4; i++)
        if (!isfinite(state[i])) return -1;
    return 1;
}

/* 前回の結果を無効化し、失敗後に古い入力を今回の結果として使わせない。 */
static void MPC_InvalidateResult(void)
{
    mpc_result_valid = 0;
    mpc_u0 = NAN;
    mpc_u1 = NAN;
    mpc_solve_time_ms = 0U;
    for (int i = 0; i < 4; i++) mpc_x0[i] = NAN;
}

/* カプセル、計算領域、モデルを一度だけ作成する。制約値更新は別関数で行う。 */
static int MPC_Init(void)
{
    if (mpc_capsule != NULL) return mpc_create_status;
    mpc_capsule = interaction_model_acados_create_capsule();
    if (mpc_capsule == NULL)
    {
        mpc_create_status = -1;
        return mpc_create_status;
    }
    mpc_create_status = interaction_model_acados_create(mpc_capsule);
    if (mpc_create_status != 0) return mpc_create_status;

    /* 制約更新にはin、計算結果の取得にはoutを使う。 */
    mpc_config = interaction_model_acados_get_nlp_config(mpc_capsule);
    mpc_dims = interaction_model_acados_get_nlp_dims(mpc_capsule);
    mpc_in = interaction_model_acados_get_nlp_in(mpc_capsule);
    mpc_out = interaction_model_acados_get_nlp_out(mpc_capsule);
    if (mpc_config == NULL || mpc_dims == NULL || mpc_in == NULL || mpc_out == NULL)
    {
        mpc_create_status = -2;
        return mpc_create_status;
    }

    /* stage 0の制約の順番をセンサのx[0],x[1],x[2],x[3]に対応させる。 */
    int idxbx[4] = {0, 1, 2, 3};
    mpc_create_status = ocp_nlp_constraints_model_set(mpc_config, mpc_dims,
                          mpc_in, mpc_out, 0, "idxbx", idxbx);
    return mpc_create_status;
}

/* 実測状態を初期状態の等式として設定する：lbx[0]=ubx[0]=実測の4状態。
   stage 1以降と終端の状態制約は、生成されたモデルの設定を維持する。 */
static int MPC_UpdateInitialState(const double measured_state[4])
{
    mpc_state_pending = 0;
    MPC_InvalidateResult();
    mpc_solve_status = -999;
    mpc_update_status = -999;
    if (mpc_create_status != 0 || mpc_capsule == NULL)
        return (mpc_update_status = -1);
    if (measured_state == NULL) return (mpc_update_status = -2);

    /* 4値すべてを検査してから制約を変更し、NaN/Infや途中更新を避ける。 */
    double state[4];
    for (int i = 0; i < 4; i++)
    {
        if (!isfinite(measured_state[i])) return (mpc_update_status = -2);
        state[i] = measured_state[i];
    }
    mpc_update_status = ocp_nlp_constraints_model_set(mpc_config, mpc_dims,
                                mpc_in, mpc_out, 0, "lbx", state);
    if (mpc_update_status != 0) return mpc_update_status;
    mpc_update_status = ocp_nlp_constraints_model_set(mpc_config, mpc_dims,
                                mpc_in, mpc_out, 0, "ubx", state);
    if (mpc_update_status != 0) return mpc_update_status;

    /* 初期状態の初期推定値も測定値に合わせる。予測系列全体は再生成しない。 */
    ocp_nlp_out_set(mpc_config, mpc_dims, mpc_out, mpc_in, 0, "x", state);
    for (int i = 0; i < 4; i++) mpc_measured_x[i] = state[i];
    mpc_state_pending = 1;
    return 0;
}

/* 入力の上下限を全予測入力へ設定する。起動時か、制約が変わったときに呼ぶ。
   1入力の現在のモデルが対象。更新はmainから行い、solveと同時に呼ばない。 */
static int MPC_UpdateInputBounds(double lower, double upper)
{
    if (mpc_create_status != 0 || mpc_capsule == NULL) return -1;
    if (!isfinite(lower) || !isfinite(upper) || lower > upper) return -2;
    /* 有効な制約変更でも、変更前の計算結果・未実行状態入力は使い回さない。 */
    mpc_state_pending = 0;
    mpc_has_warm_start = 0;
    MPC_InvalidateResult();
    int idxbu[1] = {0};
    for (int k = 0; k < INTERACTION_MODEL_N; k++)
    {
        int status = ocp_nlp_constraints_model_set(mpc_config, mpc_dims,
                                      mpc_in, mpc_out, k, "idxbu", idxbu);
        if (status != 0) return status;
        status = ocp_nlp_constraints_model_set(mpc_config, mpc_dims,
                                      mpc_in, mpc_out, k, "lbu", &lower);
        if (status != 0) return status;
        status = ocp_nlp_constraints_model_set(mpc_config, mpc_dims,
                                      mpc_in, mpc_out, k, "ubu", &upper);
        if (status != 0) return status;
    }
    mpc_input_lower = lower;
    mpc_input_upper = upper;
    return 0;
}

/* 初期推定を準備する。初回・失敗後は全予測状態を今回の測定値で初期化する。
   元の生成コードの極端な推定値x[3]=-100000を残したまま計算しない。
   成功後はLinux版と同じく前回入力列を1段ずらし、最後の入力は維持する。 */
static void MPC_PrepareWarmStart(void)
{
    /* 制約変更・失敗後は双対変数とQP内部状態もリセットする。
       引数0により、更新した制約・モデル値・オプションは保持する。 */
    if (!mpc_has_warm_start)
        interaction_model_acados_reset(mpc_capsule, 1, 0, 0, 0);
    for (int k = 0; k < INTERACTION_MODEL_N; k++)
    {
        double guess = mpc_has_warm_start
                       ? mpc_previous_u[k + 1 < INTERACTION_MODEL_N ? k + 1 : k]
                       : fmax((double)mpc_input_lower, fmin((double)mpc_input_upper, 0.0));
        ocp_nlp_out_set(mpc_config, mpc_dims, mpc_out, mpc_in, k, "u", &guess);
    }
    if (!mpc_has_warm_start)
    {
        double state[4];
        for (int i = 0; i < 4; i++) state[i] = mpc_measured_x[i];
        for (int k = 0; k <= INTERACTION_MODEL_N; k++)
            ocp_nlp_out_set(mpc_config, mpc_dims, mpc_out, mpc_in, k, "x", state);
    }
}

/* 今回の測定値で制約更新した後に、一度だけsolveを実行して結果を保存する。
   戻り値0=成功、正の値=acadosの失敗コード、負の値=入力/出力処理の失敗。 */
static int MPC_Execute(void)
{
    MPC_InvalidateResult();
    mpc_solve_status = -999;
    if (mpc_create_status != 0 || mpc_capsule == NULL)
        return (mpc_step_status = -1);
    if (!mpc_state_pending) return (mpc_step_status = -3);
    /* 同じ状態入力を、更新なしで再び計算に使わない。 */
    mpc_state_pending = 0;
    /* 予測値の準備は制約を変更する処理とは分けて行う。 */
    MPC_PrepareWarmStart();
    uint32_t start_tick = HAL_GetTick();
    mpc_solve_status = interaction_model_acados_solve(mpc_capsule);
    mpc_solve_time_ms = HAL_GetTick() - start_tick;
    mpc_iteration_count++;
    if (mpc_solve_status != 0)
    {
        mpc_failure_count++;
        mpc_has_warm_start = 0;
        return (mpc_step_status = mpc_solve_status);
    }

    /* 今回の制御入力、次段の予測入力、固定された初期状態の出力を取得する。 */
    double u0[1], u1[1], x0[4];
    ocp_nlp_out_get(mpc_config, mpc_dims, mpc_out, 0, "u", u0);
    ocp_nlp_out_get(mpc_config, mpc_dims, mpc_out, 1, "u", u1);
    ocp_nlp_out_get(mpc_config, mpc_dims, mpc_out, 0, "x", x0);
    int finite_result = isfinite(u0[0]) && isfinite(u1[0]);
    for (int i = 0; i < 4; i++) finite_result = finite_result && isfinite(x0[i]);
    /* 次回用に全入力列を取得し、非有限値の列はウォームスタートに使わない。 */
    double sequence[INTERACTION_MODEL_N];
    for (int k = 0; k < INTERACTION_MODEL_N; k++)
    {
        ocp_nlp_out_get(mpc_config, mpc_dims, mpc_out, k, "u", &sequence[k]);
        finite_result = finite_result && isfinite(sequence[k]);
    }
    if (!finite_result)
    {
        mpc_failure_count++;
        mpc_has_warm_start = 0;
        return (mpc_step_status = -4);
    }
    mpc_u0 = u0[0];
    mpc_u1 = u1[0];
    for (int i = 0; i < 4; i++) mpc_x0[i] = x0[i];
    mpc_result_sample_id = mpc_sample_count;
    for (int k = 0; k < INTERACTION_MODEL_N; k++) mpc_previous_u[k] = sequence[k];
    mpc_has_warm_start = 1;
    mpc_success_count++;
    mpc_result_valid = 1;
    return (mpc_step_status = 0);
}

/* センサ入力とソルバ出力を区別してUARTへ表示する。 */
static void MPC_PrintResult(void)
{
    printf("\r\n========== Sensor MPC ==========\r\n");
    printf("samples=%lu solves=%lu success=%lu failure=%lu\r\n",
           (unsigned long)mpc_sample_count, (unsigned long)mpc_iteration_count,
           (unsigned long)mpc_success_count, (unsigned long)mpc_failure_count);
    printf("sensor=%d update=%d solve=%d step=%d valid=%d result_sample=%lu solve_ms=%lu\r\n",
           (int)mpc_sensor_status, (int)mpc_update_status, (int)mpc_solve_status,
           (int)mpc_step_status, (int)mpc_result_valid,
           (unsigned long)mpc_result_sample_id, (unsigned long)mpc_solve_time_ms);
    if (mpc_result_valid)
    {
        printf("u0=%.10f u1=%.10f bounds=[%.6f, %.6f]\r\n",
               (double)mpc_u0, (double)mpc_u1,
               (double)mpc_input_lower, (double)mpc_input_upper);
        for (int i = 0; i < 4; i++)
            printf("measured_x[%d]=%.10f solver_x0[%d]=%.10f\r\n",
                   i, (double)mpc_measured_x[i], i, (double)mpc_x0[i]);
    }
    if (mpc_sensor_status == 0) printf("Waiting for a new sensor sample.\r\n");
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
  printf("MPC initialized. Waiting for sensor_state[4] and sensor_sample_ready=1.\r\n");
  mpc_last_report_tick = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      /* 1. センサから今回の4状態を受け取る。未着ならsolveせずに待つ。 */
      double measured_state[4];
      mpc_sensor_status = Sensors_ReadState(measured_state);
      if (mpc_sensor_status == 1)
      {
          mpc_sample_count++;
          /* 2. stage 0の下限と上限を測定値にそろえ、初期状態を固定する。 */
          mpc_update_status = MPC_UpdateInitialState(measured_state);
          if (mpc_update_status == 0)
          {
              /* 3. 更新した制約を使って、今回の制御入力を計算する。 */
              mpc_step_status = MPC_Execute();
              /* 4. 有効な計算結果を得たときだけ、実機へ今回の入力u0を渡す。 */
              if (mpc_step_status == 0 && mpc_result_valid)
              {
                  /* アクチュエータ出力を追加する位置。u1は次段の予測入力。
                     実際の出力関数・単位変換は使用する装置に合わせて実装する。 */
              }
              else
              {
                  /* 異常時の実機停止などを追加する位置。古いmpc_u0を使わない。 */
              }
          }
          else
          {
              mpc_step_status = mpc_update_status;
              mpc_failure_count++;
          }
      }
      else if (mpc_sensor_status < 0)
      {
          /* 不正なセンサ値では計算せず、前回の入力も無効にする。 */
          mpc_state_pending = 0;
          mpc_has_warm_start = 0;
          MPC_InvalidateResult();
          mpc_update_status = -2;
          mpc_solve_status = -999;
          mpc_step_status = -2;
          mpc_failure_count++;
      }

      /* 5. 初回のサンプル処理と、それ以後は約1秒ごとに状態を表示する。 */
      if ((mpc_sensor_status == 1 && mpc_sample_count == 1U) ||
          (uint32_t)(HAL_GetTick() - mpc_last_report_tick) >= MPC_REPORT_INTERVAL_MS)
      {
          MPC_PrintResult();
          mpc_last_report_tick = HAL_GetTick();
      }
      /* 6. 動作確認用の待機。計算時間も加わるため、厳密な1 ms周期ではない。 */
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
