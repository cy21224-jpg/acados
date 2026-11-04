/**
 * @file MPC_function.h
 * @brief MPC初期化・制約更新・最適化実行の公開API。
 *
 * 通常の順序：MPC_Init → MPC_UpdateInputBounds（必要時）
 * → [MPC_UpdateModelMass → MPC_UpdatePreviousAppliedInput → MPC_UpdateInitialState → MPC_Execute] を繰り返す。
 * 単一ソルバをmain側から直列に使用する。割込みや複数タスクから同時に呼ばない。
 * acadosの生成ヘッダとライブラリはMPC_function.c側で使用する。
 */
#ifndef MPC_FUNCTION_H
#define MPC_FUNCTION_H

#include "MPC_param.h"

#ifdef __cplusplus
extern "C" {
#endif

/* msの単調増加カウンタを返す関数。計算中にも進む時刻源が必要。
   同優先度の割込み内ではHAL_GetTickは進まない。今回のmainは別途DWTで計測する。
   未登録/NULLでは計算自体は実行できるが、mpc_solve_time_msは0になる。
   計算中にプロバイダを変更しない。 */
typedef uint32_t (*MPC_TickProvider)(void);
void MPC_SetTickProvider(MPC_TickProvider provider);

/* 起動時1回。0=成功。モデルのカプセルと作業領域を作成する。 */
int MPC_Init(void);
/* 再生成したNP=1のモデルのp[0]へ、正の等価質量[kg]を全予測段で設定する。
   0=成功、-1=未初期化、-2=非有限/0以下。生成APIエラーはその値を返す。
   更新時は前の結果・未実行の状態入力を無効化する。
   main側で質量更新→MPC_UpdateInitialState→MPC_Executeの順に直列実行する。
   前回の入力列はウォームスタート用に維持する。
 */
int MPC_UpdateModelMass(double equivalent_mass);
/* Update only stage-0 previous accepted force reference. NX/NU/NP unchanged.
   Call after mass update, before initial-state update. 0=success,
   -1=uninitialized, -2=nonfinite input; otherwise acados API status. */
int MPC_UpdatePreviousAppliedInput(double previous_force);
/* 読み取り検査のみ。有限性と、予測中一定のc/kの生成制約への整合を確認する。
   0=通過、-1=未初期化、-2=不正値、-5=c/kが範囲外。値を丸めない。 */
int MPC_ValidateEstimatedState(const double state[MPC_STATE_DIM]);
/* stage 0のlbx/ubxを測定値で固定。0=成功、-1=未初期化、-2=不正値、-5=c/k範囲外。 */
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
