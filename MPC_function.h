/**
 * @file MPC_function.h
 * @brief MPC初期化・制約更新・最適化実行の公開API。
 *
 * 通常の順序：MPC_Init → MPC_UpdateInputBounds（必要時）
 * → [MPC_UpdateInitialState → MPC_Execute] を各サンプルで繰り返す。
 * 単一ソルバをmain側から直列に使用する。割込みや複数タスクから同時に呼ばない。
 * acadosの生成ヘッダとライブラリはMPC_function.c側で使用する。
 */
#ifndef MPC_FUNCTION_H
#define MPC_FUNCTION_H

#include "MPC_param.h"

#ifdef __cplusplus
extern "C" {
#endif

/* msの単調増加カウンタを返す関数。STM32ではHAL_GetTickを登録する。
   未登録/NULLでは計算自体は実行できるが、mpc_solve_time_msは0になる。
   計算中にプロバイダを変更しない。 */
typedef uint32_t (*MPC_TickProvider)(void);
void MPC_SetTickProvider(MPC_TickProvider provider);

/* 起動時1回。0=成功。モデルのカプセルと作業領域を作成する。 */
int MPC_Init(void);
/* stage 0のlbx/ubxを測定値で固定。0=成功、-1=未初期化、-2=不正値。 */
int MPC_UpdateInitialState(const double measured_state[MPC_STATE_DIM]);
/* 全予測入力の上下限を更新。成功後は再度初期状態を更新してからExecuteする。 */
int MPC_UpdateInputBounds(double lower, double upper);
/* 新しい初期状態に対して1回計算し、mpc_u0/u1/x0へ保存する。
   0=成功、正の値=acadosの失敗、-1=未初期化、-3=状態未更新、-4=非有限出力。 */
int MPC_Execute(void);
/* 結果だけをNaN/valid=0に戻す。制約値・ウォームスタートは維持する。 */
void MPC_InvalidateResult(void);
/* 不正なセンササンプル等を破棄する。未実行状態とウォームスタートも無効化する。
   呼び出し側のステータス記録・失敗回数加算はmainで行う。 */
void MPC_DiscardState(void);

#ifdef __cplusplus
}
#endif

#endif /* MPC_FUNCTION_H */
