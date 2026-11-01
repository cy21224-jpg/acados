/**
 * @file param_3link.h
 * @brief param_3link.cに存在する3リンク装置の変数を宣言する。
 * 配列の型・要素数はCファイルと一致させ、実体はこのヘッダーに置かない。
 * main.cから変数を移すときは、同名の実体定義をmain.cに重複させないこと。
 */
#ifndef PARAM_3LINK_H
#define PARAM_3LINK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 制御周期・ゲイン・装置定数。元の設定値を維持する。 */
extern const float sampling_time;
extern const float t_start;
extern const float t_end;
extern const float pi;
extern const float g_vm[3][3];
extern const float g_dob[3][3];
extern const float g_rtob[3][3];
extern const float g_wob[3][3];
extern const float Kpp_w[3][3];
extern const float Kpp_j[3][3];
extern const float Kpd_w[3][3];
extern const float Kpd_j[3][3];
extern const float Kfp_w;
extern const float Kfp[3][3];
extern const float hat_D_frc_mns[3];
extern const float hat_D_frc_pls[3];
extern const float hat_tau_frc_mns[3];
extern const float hat_tau_frc_pls[3];
extern const float I[3][3];
extern const float S[3][3];
extern const float Jmn[3][3];
extern const float Gr[3][3];
extern const float Ktn[3][3];
extern const float Jn[3][3];
extern const float L1;
extern const float L2;
extern const float L3;
extern const float M1;
extern const float M2;
extern const float M3;
extern float I_z1;
extern float I_z2;
extern float I_z3;

/* モータ制御・座標変換・オブザーバで使用する共有変数。 */
extern int cnt_time;
extern int overflowcnt[3];
extern uint32_t cnt_encoder[3];
extern uint16_t D_out[3];
extern float th_res_deg[3];
extern float th_res[3];
extern float dth_res[3];
extern float signal_dth_res[3];
extern float t;
extern int i;
extern int j;
extern int k;
extern int flg_time;
extern float i_a_ref[3];
extern float th_cmd[3];
extern float dth_cmd[3];
extern float ddth_ref[3];
extern float tau_ref[3];
extern float i_ref[3];
extern float signal_dob1[3];
extern float signal_dob2[3];
extern float hat_tau_dis[3];
extern float tau_cmd[3];
extern int flag_init;
extern float x_w_cmd[3];
extern float F_w_cmd[3];
extern float dx_w_cmd[3];
extern float ddx_w_ref[3];
extern float ddx_wp_ref[3];
extern float ddx_wf_ref[3];
extern float signal_rtob1[3];
extern float signal_rtob2[3];
extern float hat_tau_env[3];
extern float signal_wob1[3];
extern float signal_wob2[3];
extern float tilde_ddx_w_ref[3];
extern float dx_w_res[3];
extern float x_w_res[3];
extern float hat_ddx_w_dis[3];
extern float hat_f_env[3];
extern float Mn[3][3];
extern float J_aco[3][3];
extern float J_aco_inv[3][3];
extern float det_J_aco;
extern float J_aco_T[3][3];
extern float J_aco_T_inv[3][3];
extern float det;
extern uint8_t cmd1[2];
extern uint8_t cmd2[2];

/* EKF: 4状態[v_y, y, c_env, k_env]、観測[v_y, y]、入力4個。
 * M_wy_k_1は正の有効質量、P/R/Qは設計した共分散を呼出し前に設定する。
 * 添付で初期値のなかった変数は、元どおり0初期化。任意の値を補わない。
 */
extern double y_k[2];
extern double u_k_1[4];
extern double x_hat_k_1[4];
extern double P_k_1[4][4];
extern double Rkf[2][2];
extern double Qkf[4][4];
extern double x_hat_k[4];
extern double P_k[4][4];
extern double M_wy_k_1;

#ifdef __cplusplus
}
#endif

#endif /* PARAM_3LINK_H */
