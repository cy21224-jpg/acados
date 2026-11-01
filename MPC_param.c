/** @file MPC_param.c
 *  @brief MPC設定に基づく変数の初期値と、共有する計算結果・ステータスの実体。
 *  デフォルト入力範囲を変えたら、初期化後にMPC_UpdateInputBounds()で反映する。
 */
#include "MPC_param.h"
#include <math.h>

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
volatile double mpc_measured_x[MPC_STATE_DIM] = {NAN, NAN, NAN, NAN};
volatile double mpc_x0[MPC_STATE_DIM] = {NAN, NAN, NAN, NAN};
volatile double mpc_u0 = NAN;
volatile double mpc_u1 = NAN;

