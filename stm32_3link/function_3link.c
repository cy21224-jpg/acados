/**
 * @file function_3link.c
 * @brief 添付のRIM_functionsを使用する、3リンク装置用の4状態・4入力EKF。
 * double型と入力4成分を元の関数から維持し、param_3link.c/.hと一致させる。
 * HALやタイマ設定、動的メモリ、静的な共有作業領域を使用しない。
 */
#include "function_3link.h"
#include "param_3link.h"
#include "RIM_functions.h"

#include <float.h>
#include <math.h>
#include <stddef.h>

/* 入力共分散の有限性・対称性・半正定値性を確認する。
 * 丸め範囲の非対称性は内部コピーだけを対称化し、入力配列は変更しない。
 */
static int covariance_is_valid(int dimension, double matrix[dimension][dimension])
{
    double scale = 0.0;
    double factor[4][4] = {{0.0}};
    for (int row = 0; row < dimension; ++row) {
        for (int col = 0; col < dimension; ++col) {
            if (!isfinite(matrix[row][col])) return 0;
            scale = fmax(scale, fabs(matrix[row][col]));
        }
        if (matrix[row][row] < 0.0) return 0;
    }
    if (scale == 0.0) return 1;
    for (int row = 0; row < dimension; ++row) {
        for (int col = 0; col < row; ++col) {
            if (fabs(matrix[row][col] - matrix[col][row]) >
                64.0 * DBL_EPSILON * scale) return 0;
            const double average = 0.5 * (matrix[row][col] + matrix[col][row]);
            matrix[row][col] = average;
            matrix[col][row] = average;
        }
    }
    for (int row = 0; row < dimension; ++row) {
        for (int col = 0; col <= row; ++col) {
            double value = matrix[row][col] / scale;
            for (int index = 0; index < col; ++index)
                value -= factor[row][index] * factor[col][index];
            if (row == col) {
                if (value < -128.0 * DBL_EPSILON) return 0;
                factor[row][col] = sqrt(fmax(value, 0.0));
            } else if (factor[col][col] > 0.0) {
                factor[row][col] = value / factor[col][col];
            } else if (fabs(value) > 128.0 * DBL_EPSILON) {
                return 0;
            }
        }
    }
    return 1;
}

/* 1サンプルを予測・観測更新する。失敗時は呼出し側の出力を変更しない。 */
EKF_Status EKF(double measurement[2], double previous_input[4], double mass,
               double previous_state[4], double previous_covariance[4][4],
               double measurement_noise[2][2], double process_noise[4][4],
               double estimated_state[4], double estimated_covariance[4][4])
{
    if (measurement == NULL || previous_input == NULL || previous_state == NULL ||
        previous_covariance == NULL || measurement_noise == NULL ||
        process_noise == NULL || estimated_state == NULL ||
        estimated_covariance == NULL) return EKF_STATUS_BAD_ARGUMENT;
    if (!isfinite(mass) || mass <= 0.0) return EKF_STATUS_INVALID_MASS;
    const double dt = (double)sampling_time;
    if (!isfinite(dt) || dt <= 0.0) return EKF_STATUS_BAD_ARGUMENT;

    /* 入出力を同じ配列にする場合にも対応するため、前回値を先にコピーする。 */
    double state[4], input[4], observed[2];
    double covariance[4][4], q[4][4], r[2][2];
    copy_vec(4, previous_state, state);
    copy_vec(4, previous_input, input);
    copy_vec(2, measurement, observed);
    copy_mat(4, 4, previous_covariance, covariance);
    copy_mat(4, 4, process_noise, q);
    copy_mat(2, 2, measurement_noise, r);
    for (int index = 0; index < 4; ++index)
        if (!isfinite(state[index]) || !isfinite(input[index]))
            return EKF_STATUS_BAD_ARGUMENT;
    for (int index = 0; index < 2; ++index)
        if (!isfinite(observed[index])) return EKF_STATUS_BAD_ARGUMENT;
    if (!covariance_is_valid(4, covariance) || !covariance_is_valid(4, q) ||
        !covariance_is_valid(2, r)) return EKF_STATUS_INVALID_COVARIANCE;

    /* 非線形予測f(x)とヤコビアンFを同じモデルから作る。
     * 元コードの速度予測の末尾「+位置」は「+速度」へ修正する。
     * B*uの4入力と、G*Q*G^Tの雑音スケーリングは元の式を維持する。
     */
    const double h = dt / mass;
    double transition[4][4] = {
        {1.0 - h * state[2], -h * state[3], -h * state[0], -h * state[1]},
        {dt, 1.0, 0.0, 0.0},
        {0.0, 0.0, 1.0, 0.0},
        {0.0, 0.0, 0.0, 1.0}
    };
    double input_matrix[4][4] = {
        {h, 0.0, 0.0, 0.0},
        {0.0, dt, 0.0, 0.0},
        {0.0, 0.0, dt, 0.0},
        {0.0, 0.0, 0.0, dt}
    };
    double observation[2][4] = {{1.0, 0.0, 0.0, 0.0}, {0.0, 1.0, 0.0, 0.0}};
    double nonlinear_state[4] = {
        state[0] - h * (state[2] * state[0] + state[3] * state[1]),
        state[1] + dt * state[0], state[2], state[3]
    };
    double input_effect[4], predicted_state[4];
    mat_vec_mul(4, 4, input_matrix, input, input_effect);
    add_vec(4, nonlinear_state, input_effect, predicted_state);

    /* P_minus = F*P*F^T + B*Q*B^T。不要になった作業行列は後の段階で再利用する。 */
    double temporary[4][4], transposed[4][4];
    double predicted_covariance[4][4], scratch[4][4];
    if (mul_mat(4, 4, 4, 4, transition, covariance, temporary) != 0)
        return EKF_STATUS_NUMERIC_ERROR;
    transpose_mat(4, 4, transition, transposed);
    if (mul_mat(4, 4, 4, 4, temporary, transposed, predicted_covariance) != 0)
        return EKF_STATUS_NUMERIC_ERROR;
    if (mul_mat(4, 4, 4, 4, input_matrix, q, temporary) != 0)
        return EKF_STATUS_NUMERIC_ERROR;
    transpose_mat(4, 4, input_matrix, transposed);
    if (mul_mat(4, 4, 4, 4, temporary, transposed, scratch) != 0)
        return EKF_STATUS_NUMERIC_ERROR;
    add_mat(4, 4, predicted_covariance, scratch, predicted_covariance);
    for (int row = 0; row < 4; ++row) {
        if (!isfinite(predicted_state[row])) return EKF_STATUS_NUMERIC_ERROR;
        for (int col = 0; col < 4; ++col)
            if (!isfinite(predicted_covariance[row][col])) return EKF_STATUS_NUMERIC_ERROR;
        for (int col = 0; col < row; ++col) {
            const double average = 0.5 * (predicted_covariance[row][col] +
                                          predicted_covariance[col][row]);
            predicted_covariance[row][col] = average;
            predicted_covariance[col][row] = average;
        }
    }

    /* 観測共分散S = C*P_minus*C^T + Rを作る。 */
    double observation_transposed[4][2], pc[4][2], cp[2][4];
    double innovation_covariance[2][2], scaled_innovation[2][2], inverse_scaled[2][2];
    transpose_mat(2, 4, observation, observation_transposed);
    if (mul_mat(4, 4, 4, 2, predicted_covariance, observation_transposed, pc) != 0 ||
        mul_mat(2, 4, 4, 4, observation, predicted_covariance, cp) != 0 ||
        mul_mat(2, 4, 4, 2, cp, observation_transposed, innovation_covariance) != 0)
        return EKF_STATUS_NUMERIC_ERROR;
    add_mat(2, 2, innovation_covariance, r, innovation_covariance);

    /* inv_matは絶対ピボット閾値を使うため、Sを正規化してから渡す。
     * 特異・極端に悪条件な場合は、UARTへのエラー表示を伴うinv_matを呼ぶ前に戻る。
     */
    double scale = 0.0;
    for (int row = 0; row < 2; ++row)
        for (int col = 0; col < 2; ++col) {
            if (!isfinite(innovation_covariance[row][col])) return EKF_STATUS_NUMERIC_ERROR;
            scale = fmax(scale, fabs(innovation_covariance[row][col]));
        }
    if (scale == 0.0 || innovation_covariance[0][0] <= 0.0 ||
        innovation_covariance[1][1] <= 0.0) return EKF_STATUS_SINGULAR_INNOVATION;
    /* 1/scaleを先に計算せず直接割り、極小の共分散でも不要なオーバーフローを避ける。 */
    for (int row = 0; row < 2; ++row)
        for (int col = 0; col < 2; ++col)
            scaled_innovation[row][col] = innovation_covariance[row][col] / scale;
    const double a = scaled_innovation[0][0];
    const double b = scaled_innovation[0][1];
    const double d = scaled_innovation[1][1];
    const double determinant = a * d - b * b;
    if (determinant <= fmax(1e-14, 64.0 * DBL_EPSILON * (a * d + b * b)))
        return EKF_STATUS_SINGULAR_INNOVATION;
    if (inv_mat(2, scaled_innovation, inverse_scaled) != 0)
        return EKF_STATUS_SINGULAR_INNOVATION;

    /* K = (P_minus*C^T / scale) * inv(S/scale)。スケールを相殺してから積を取る。 */
    double gain[4][2];
    for (int row = 0; row < 4; ++row)
        for (int col = 0; col < 2; ++col) pc[row][col] /= scale;
    if (mul_mat(4, 2, 2, 2, pc, inverse_scaled, gain) != 0)
        return EKF_STATUS_NUMERIC_ERROR;
    double predicted_observation[2], residual[2], state_correction[4], corrected_state[4];
    mat_vec_mul(2, 4, observation, predicted_state, predicted_observation);
    sub_vec(2, observed, predicted_observation, residual);
    mat_vec_mul(4, 2, gain, residual, state_correction);
    add_vec(4, predicted_state, state_correction, corrected_state);

    /* Joseph形式で共分散を更新する。
     * P = (I-KC)*P_minus*(I-KC)^T + K*R*K^T。
     * 厳密演算では元の(I-KC)*P_minusと同じ。作業領域を再利用してスタックを抑える。
     */
    eye_mat(4, transition);
    if (mul_mat(4, 2, 2, 4, gain, observation, scratch) != 0)
        return EKF_STATUS_NUMERIC_ERROR;
    sub_mat(4, 4, transition, scratch, transition);
    if (mul_mat(4, 4, 4, 4, transition, predicted_covariance, temporary) != 0)
        return EKF_STATUS_NUMERIC_ERROR;
    transpose_mat(4, 4, transition, transposed);
    if (mul_mat(4, 4, 4, 4, temporary, transposed, scratch) != 0 ||
        mul_mat(4, 2, 2, 2, gain, r, pc) != 0)
        return EKF_STATUS_NUMERIC_ERROR;
    transpose_mat(4, 2, gain, cp);
    if (mul_mat(4, 2, 2, 4, pc, cp, predicted_covariance) != 0)
        return EKF_STATUS_NUMERIC_ERROR;
    add_mat(4, 4, scratch, predicted_covariance, scratch);

    /* 状態・共分散が有限であることを確認し、成功時だけ出力をまとめて更新する。 */
    for (int row = 0; row < 4; ++row) {
        if (!isfinite(corrected_state[row])) return EKF_STATUS_NUMERIC_ERROR;
        for (int col = 0; col < 4; ++col) {
            if (!isfinite(scratch[row][col])) return EKF_STATUS_NUMERIC_ERROR;
            if (col < row) {
                const double average = 0.5 * scratch[row][col] + 0.5 * scratch[col][row];
                scratch[row][col] = average;
                scratch[col][row] = average;
            }
        }
    }
    copy_vec(4, corrected_state, estimated_state);
    copy_mat(4, 4, scratch, estimated_covariance);
    return EKF_STATUS_OK;
}

/* 既存APIを維持する。EKF設計値を補ったり、運転中の制御状態をリセットしたりしない。 */
void param_init(void)
{
    /* 各変数はparam_3link.cで静的初期化される。EKF設計値は呼出し側で設定する。 */
}
