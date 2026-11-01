/**
 * @file MPC_function.c
 * @brief MPCの生成ソルバを扱う処理。HAL・GPIO・UART・センサ取得には依存しない。
 *
 * 別の生成モデルへ移す場合は、以下の生成ヘッダ・シンボル名と次元確認を更新する。
 * 作業領域は初期化時に作成し、計算ループ中にカプセルを作り直さない。
 */
#include "MPC_function.h"
#include "acados_solver_interaction_model.h"
#include <math.h>
#include <stddef.h>

_Static_assert(MPC_STATE_DIM == 4 && MPC_INPUT_DIM == 1 &&
               INTERACTION_MODEL_NX == MPC_STATE_DIM &&
               INTERACTION_MODEL_NU == MPC_INPUT_DIM &&
               INTERACTION_MODEL_NBX0 == MPC_STATE_DIM &&
               INTERACTION_MODEL_NBU == MPC_INPUT_DIM &&
               INTERACTION_MODEL_N >= 2,
               "Update MPC adapter for the generated model dimensions");

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

static MPC_TickProvider mpc_tick_provider = NULL;

/* 実装詳細のウォームスタート準備関数は、公開APIからだけ呼ぶ。 */
static void MPC_PrepareWarmStart(void);

/* 時刻取得を注入し、HALをMPC本体に持ち込まない。 */
void MPC_SetTickProvider(MPC_TickProvider provider)
{
    mpc_tick_provider = provider;
}

static uint32_t MPC_GetTick(void)
{
    return mpc_tick_provider != NULL ? mpc_tick_provider() : 0U;
}

/* 前回の結果を無効化し、失敗後に古い入力を今回の結果として使わせない。 */
void MPC_InvalidateResult(void)
{
    mpc_result_valid = 0;
    mpc_u0 = NAN;
    mpc_u1 = NAN;
    mpc_solve_time_ms = 0U;
    for (int i = 0; i < 4; i++) mpc_x0[i] = NAN;
}

/* カプセル、計算領域、モデルを一度だけ作成する。制約値更新は別関数で行う。 */
int MPC_Init(void)
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
int MPC_UpdateInitialState(const double measured_state[MPC_STATE_DIM])
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
int MPC_UpdateInputBounds(double lower, double upper)
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
int MPC_Execute(void)
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
    uint32_t start_tick = MPC_GetTick();
    mpc_solve_status = interaction_model_acados_solve(mpc_capsule);
    mpc_solve_time_ms = MPC_GetTick() - start_tick;
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


/* センサ不正時の内部状態破棄をAPIにまとめ、mainからprivate変数を触らせない。 */
void MPC_DiscardState(void)
{
    mpc_state_pending = 0;
    mpc_has_warm_start = 0;
    MPC_InvalidateResult();
}
