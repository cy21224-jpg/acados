/**
 * @file MPC_param.h
 * @brief MPCの初期設定値と、共有する状態・計算結果の宣言。
 *
 * 変数の実体はMPC_param.cに1つだけ置く。ヘッダ内に実体を定義しない。
 * 予測段数・モデル・評価重み・刻み時間などPythonから生成する値は、
 * acados生成ファイルで管理する。このヘッダで二重に定義しない。
 */
#ifndef MPC_PARAM_H
#define MPC_PARAM_H

#include <stdint.h>

/* 現在のモデルは4状態・1入力。変更時はモデル側との整合性を確認する。 */
#define MPC_STATE_DIM 4
#define MPC_INPUT_DIM 1
/* アプリケーションの待機・表示間隔。ソルバの予測刻みとは別の設定。 */
#define MPC_LOOP_DELAY_MS 1U
#define MPC_REPORT_INTERVAL_MS 1000U
/* 入力制約の初期値。実際の変更にはMPC_UpdateInputBounds()を使う。 */
#define MPC_INPUT_MIN 0.0
#define MPC_INPUT_MAX 10.0

#ifdef __cplusplus
extern "C" {
#endif

/* 共有ステータス・結果。直接書き換えてもソルバのモデル値は変わらない。
   -999=未実行。sensor_statusは0=未着、1=取得、-1=非有限値。
   sample_countはmain側で採番し、Executeはその番号を結果へ記録する。 */
/* 最後に設定した質量と更新結果。未設定はNaN / -999。 */
extern volatile double mpc_model_mass;
extern volatile int mpc_mass_update_status;
extern volatile int mpc_create_status;
extern volatile int mpc_sensor_status;
extern volatile int mpc_update_status;
extern volatile int mpc_step_status;
extern volatile int mpc_solve_status;
extern volatile uint32_t g_heap_peak_bytes;
extern volatile uint32_t mpc_sample_count;
extern volatile uint32_t mpc_iteration_count;
extern volatile uint32_t mpc_success_count;
extern volatile uint32_t mpc_failure_count;
extern volatile int mpc_result_valid;
extern volatile uint32_t mpc_result_sample_id;
extern volatile uint32_t mpc_solve_time_ms;
extern volatile double mpc_input_lower;
extern volatile double mpc_input_upper;
extern volatile double mpc_measured_x[MPC_STATE_DIM];
extern volatile double mpc_x0[MPC_STATE_DIM];
extern volatile double mpc_u0;
extern volatile double mpc_u1;

#ifdef __cplusplus
}
#endif

#endif /* MPC_PARAM_H */
