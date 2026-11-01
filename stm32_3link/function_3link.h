/**
 * @file function_3link.h
 * @brief STM32で使用する4状態・4入力EKFの公開API。
 * 行列計算には添付のRIM_functions.c/.hを使用する。HAL・タイマ設定には依存しない。
 */
#ifndef FUNCTION_3LINK_H
#define FUNCTION_3LINK_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    EKF_STATUS_OK = 0,
    EKF_STATUS_BAD_ARGUMENT = -1,
    EKF_STATUS_INVALID_MASS = -2,
    EKF_STATUS_INVALID_COVARIANCE = -3,
    EKF_STATUS_SINGULAR_INNOVATION = -4,
    EKF_STATUS_NUMERIC_ERROR = -5
} EKF_Status;

/**
 * 1サンプル分の拡張カルマンフィルタを実行する。
 *
 * 状態 = [速度, 位置, 環境粘性, 環境剛性]、観測 = [速度, 位置]。
 * u_k_1は直前区間の入力4個、M_wy_k_1は正の有効質量。
 * 予測式: x_next = f(x) + B*u、B = diag(dt/M, dt, dt, dt)。
 * f(x) = [v - dt*(c*v+k*y)/M, y+dt*v, c, k]。
 * 入力[0]は速度方程式への力、入力[1..3]は位置・粘性・剛性への駆動項。
 * 駆動しない成分は呼出し側で0にする。4成分を自動的に同じ値にしない。
 * dtはparam_3link.cのsampling_time。元の1 msを維持する。
 * Qkfは連続雑音側の4x4共分散として、元の式G*Qkf*G^Tを使う。
 * G = diag(dt/M, dt, dt, dt)。Rkfは2x2観測雑音共分散。
 *
 * 型は元の関数とRIM_functionsに合わせてdoubleとし、param_3link側も統一する。
 * 行列引数はC11の多次元配列の互換性のためconstを付けていないが、
 * 入力行列は書き換えない。出力は成功時だけまとめて更新する。
 * x_hat_k_1==x_hat_k、P_k_1==P_kによるインプレース更新にも対応する。
 * 出力状態配列と出力共分散行列を同じ領域に重ねてはならない。
 * 失敗時は出力を変更しないため、戻り値を確認して古い出力と区別する。
 *
 * EKFは固定長の自動変数のみ。RIM_functionsのinv_matにはN=2を渡す。
 * mallocや静的な共有作業領域は使わない。
 * 呼出し中に入力や出力を別の割込み・タスクから変更しないこと。
 * 質量・P/R/Qの設計値は呼出し側が設定し、次回用の状態・共分散も保持する。
 */
EKF_Status EKF(double y_k[2], double u_k_1[4], double M_wy_k_1,
               double x_hat_k_1[4], double P_k_1[4][4],
               double Rkf[2][2], double Qkf[4][4],
               double x_hat_k[4], double P_k[4][4]);

/**
 * 元のparam_init()との互換用。
 * パラメータは静的初期化済みなので何も変更しない。
 * EKFの質量・初期推定値・P/R/Qを自動設定したり、制御状態をリセットしない。
 */
void param_init(void);

#ifdef __cplusplus
}
#endif

#endif /* FUNCTION_3LINK_H */
