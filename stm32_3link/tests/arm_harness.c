#include "param_3link.h"
#include "function_3link.h"
volatile int arm_ekf_status = -999;
void arm_ekf_step(void)
{
    arm_ekf_status = EKF(y_k, u_k_1, M_wy_k_1, x_hat_k_1, P_k_1,
                         Rkf, Qkf, x_hat_k, P_k);
}
void Reset_Handler(void)
{
    arm_ekf_step();
    for (;;) { }
}
