/**
 * @file param_3link.c
 * @brief 3リンク装置のパラメータと共有変数の実体。Core/Srcに1個だけ置く。
 * モータ制御の数値・型・配列サイズは添付のparam_3link.cを維持する。
 * EKF変数だけ関数・RIM_functionsのdoubleにそろえ、u_k_1は指定どおり4要素とする。
 * 派生定数はCの静的初期化に使える定数式へ展開している。
 * タイマ、クロック、MPCソルバの設定は変更しない。
 */
#include "param_3link.h"

/* 相互に依存する静的初期値を1か所で管理する。
 * const変数同士の参照はISO Cの定数式にならないため、ここだけマクロを使う。
 * Kfp_w/Gr/Jmn/M1..3を調整するときは対応する値をここで変更する。
 */
#define PARAM3_KFP_W_VALUE 1.0f
#define PARAM3_GR1_VALUE 100.0f
#define PARAM3_GR2_VALUE 21.0f
#define PARAM3_GR3_VALUE 1.0f
#define PARAM3_JMN1_VALUE 0.81e-4f
#define PARAM3_JMN2_VALUE 142.0e-7f
#define PARAM3_JMN3_VALUE 34.7e-7f
#define PARAM3_M1_VALUE 2.30f
#define PARAM3_M2_VALUE 1.5f
#define PARAM3_M3_VALUE 0.06f


/* Control time parameters */
const float sampling_time = 1.0e-3f;
const float t_start = 5.0f;
const float t_end = 100.0f;

const float pi = 3.14f;
/* END */

/* Control parameters */
const float g_vm[3][3] = {{ 30.0f,    0.0f,     0.0f},
		                  {  0.0f,  100.0f,     0.0f},
		                  {  0.0f,    0.0f,   100.0f}};//Cutoff frequency of angular velocity measurement
const float g_dob[3][3] = {{ 20.0f*0.5f,         0.0f,          0.0f},
                           {       0.0f,  100.0f*0.5f,          0.0f},
                           {       0.0f,         0.0f,   100.0f*0.5f}};//Cutoff frequency of Disturbance Observer
const float g_rtob[3][3] = {{ 20.0f*0.5f,         0.0f,          0.0f},
                            {       0.0f,  100.0f*0.5f,          0.0f},
                            {       0.0f,         0.0f,   100.0f*0.5f}};//Cutoff frequency of Disturbance Observer
const float g_wob[3][3] = {{0.0f,  0.0f, 0.0f},
                          { 0.0f,  0.0f, 0.0f},
                          { 0.0f,  0.0f, 0.0f}};//Cutoff frequency of workspace Observer

const float Kpp_w[3][3] = {{ 10.0f,  0.0f,   0.0f},
						 { 0.0f,  10.0f,   0.0f},
						 { 0.0f,  0.0f,   20.0f}};
const float Kpp_j[3][3] = {{ 10.0f,  0.0f,   0.0f},
						 { 0.0f,  10.0f,   0.0f},
						 { 0.0f,  0.0f,   100.0f}};

const float Kpd_w[3][3] = {{6.324555320336758663998f,             0.0f,             0.0f},
						 {            0.0f,      6.324555320336758663998f,          0.0f},
						 {            0.0f,             0.0f,                8.944271909999158785637f}};

const float Kpd_j[3][3] = {{6.324555320336758663998f,             0.0f,             0.0f},
						 {            0.0f, 6.324555320336758663998f,             0.0f},
						 {            0.0f,             0.0f, 20.0f}};

const float Kfp_w = PARAM3_KFP_W_VALUE;
const float Kfp[3][3] = {{PARAM3_KFP_W_VALUE, 0.0f, 0.0f},
                            {0.0f, PARAM3_KFP_W_VALUE, 0.0f},
                            {0.0f, 0.0f, PARAM3_KFP_W_VALUE}};
const float hat_D_frc_mns[3] = {12.42f, 0.0f, 0.0f};
const float hat_D_frc_pls[3] = {12.42f, 0.0f, 0.0f};
const float hat_tau_frc_mns[3] = {-5.2731f ,0.0f, 0.0f};
const float hat_tau_frc_pls[3] = {5.2731f ,0.0f, 0.0f};
const float I[3][3] = {{1.0f, 0.0f, 0.0f},
					   {0.0f, 1.0f, 0.0f},
					   {0.0f, 0.0f, 1.0f}};
const float S[3][3] = {{1.0f, 0.0f, 0.0f},
					   {0.0f, 0.0f, 0.0f},
					   {0.0f, 0.0f, 0.0f}};
/* END */



/* Physical parameters */
const float Jmn[3][3] = {{PARAM3_JMN1_VALUE, 0.0f, 0.0f},
                         {0.0f, PARAM3_JMN2_VALUE, 0.0f},
                         {0.0f, 0.0f, PARAM3_JMN3_VALUE}};
const float Gr[3][3] = {{PARAM3_GR1_VALUE, 0.0f, 0.0f},
                        {0.0f, PARAM3_GR2_VALUE, 0.0f},
                        {0.0f, 0.0f, PARAM3_GR3_VALUE}};
const float Ktn[3][3] = {{5.76e-2f,     0.0f,     0.0f},
						 { 0.0f, 30.2e-3f,     0.0f},
						 { 0.0f,     0.0f, 53.8e-3f}};
const float Jn[3][3] = {{PARAM3_GR1_VALUE * PARAM3_GR1_VALUE * PARAM3_JMN1_VALUE,                             0.0,                            0.0},
                  {                            0.0, PARAM3_GR2_VALUE * PARAM3_GR2_VALUE * PARAM3_JMN2_VALUE,                             0.0},
				  {                            0.0,                             0.0, PARAM3_GR3_VALUE * PARAM3_GR3_VALUE * PARAM3_JMN3_VALUE}};

const float L1 = 0.19f;
const float L2 = 0.19f;
const float L3 = 0.19f;

const float M1=PARAM3_M1_VALUE;//0.235f;
const float M2=PARAM3_M2_VALUE;//0.235+0.5f;
const float M3=PARAM3_M3_VALUE;

float I_z1=PARAM3_M1_VALUE*(0.065f+0.0049f)/12.0f;
float I_z2=PARAM3_M2_VALUE*(0.065f+0.0049f)/12.0f;
float I_z3=PARAM3_M3_VALUE*(0.065f+0.0049f)/12.0f;

/* Other parameters */

//Motor control variables
int cnt_time = 0;
int overflowcnt[3] = {0, 0, 0};
uint32_t cnt_encoder[3] = {0, 0, 0};
uint16_t D_out[3] = {2048, 2048, 2048};
float th_res_deg[3] = {0.0f, 0.0f, 0.0f};
float th_res[3] = {0.0f, 0.0f, 0.0f};
float dth_res[3] = {0.0f, 0.0f, 0.0f};
float signal_dth_res[3] = {0.0f, 0.0f, 0.0f};

float t = 0.0f;
int i = 0;
int j = 0;
int k = 0;
int flg_time = 0;

float i_a_ref[3] = {0.0f,0.0f,0.0f};
float th_cmd[3] = {0.0f,0.0f,0.0f};
float dth_cmd[3] = {0.0f,0.0f,0.0f};
float ddth_ref[3] = {0.0f,0.0f,0.0f};
float tau_ref[3] = {0.0f,0.0f,0.0f};
float i_ref[3] = {0.0f,0.0f,0.0f};
float signal_dob1[3] = {0.0f, 0.0f, 0.0f};
float signal_dob2[3] = {0.0f, 0.0f, 0.0f};
float hat_tau_dis[3] = {0.0f, 0.0f, 0.0f};
float tau_cmd[3] = {0.0f, 0.0f, 0.0f};
int flag_init=0;

float x_w_cmd[3] = {0.0f,0.0f,0.0f};
float F_w_cmd[3] = {0.0f,0.0f,0.0f};
float dx_w_cmd[3] = {0.0f,0.0f,0.0f};
float ddx_w_ref[3] = {0.0f,0.0f,0.0f};
float ddx_wp_ref[3] = {0.0f,0.0f,0.0f};
float ddx_wf_ref[3] = {0.0f,0.0f,0.0f};
float signal_rtob1[3] = {0.0f, 0.0f, 0.0f};
float signal_rtob2[3] = {0.0f, 0.0f, 0.0f};
float hat_tau_env[3] = {0.0f, 0.0f, 0.0f};
float signal_wob1[3] = {0.0f, 0.0f, 0.0f};
float signal_wob2[3] = {0.0f, 0.0f, 0.0f};
float tilde_ddx_w_ref[3] = {0.0f, 0.0f, 0.0f};
float dx_w_res[3] = {0.0f, 0.0f, 0.0f};
float x_w_res[3] = {0.0f, 0.0f, 0.0f};
float hat_ddx_w_dis[3] = {0.0f, 0.0f, 0.0f};
float hat_f_env[3] = {0.0f, 0.0f, 0.0f};
float Mn[3][3] = {{0.0f,0.0f,0.0f},{0.0f,0.0f,0.0f},{0.0f,0.0f,0.0f}};
float J_aco[3][3] = {{0.0f,0.0f,0.0f},{0.0f,0.0f,0.0f},{0.0f,0.0f,0.0f}};
float J_aco_inv[3][3] = {{0.0f,0.0f,0.0f},{0.0f,0.0f,0.0f},{0.0f,0.0f,0.0f}};
float det_J_aco=0.0f;
float J_aco_T[3][3];
float J_aco_T_inv[3][3];
float det;
uint8_t cmd1[2]={0.0};
uint8_t cmd2[2]={0.0};
/* END */


/*EKF*/
double y_k[2];
double u_k_1[4];
double x_hat_k_1[4];
double P_k_1[4][4];
double Rkf[2][2];
double Qkf[4][4];
double x_hat_k[4];
double P_k[4][4];
double M_wy_k_1;
