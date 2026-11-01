/** @file MPC_timing.h
 *  @brief STM32F767のDWTによる実行時間計測。既存TIM設定を変更しない。
 *  mainのUSER CODE領域から使用する任意の計測補助。
 */
#ifndef MPC_TIMING_H
#define MPC_TIMING_H

#include "stm32f7xx.h"
#include <stdint.h>

/* クロック設定完了後、起動時に1回だけ呼ぶ。0=使用可能、-1=使用不可。
 * CPUサイクルカウンタだけを初期化し、TIM/割込み優先順位/CPU周波数は変えない。
 */
static inline int MPC_CycleCounterInit(void)
{
    if (SystemCoreClock == 0U || (DWT->CTRL & DWT_CTRL_NOCYCCNT_Msk) != 0U)
        return -1;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->LAR = 0xC5ACCE55UL;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    __DSB();
    __ISB();
    return (DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U ? 0 : -1;
}

/* 終了時-開始時をuint32_tで引く。カウンタ1周未満の区間を計測する。 */
static inline uint32_t MPC_CycleCounterRead(void)
{
    return DWT->CYCCNT;
}

/* CPUサイクル数をusへ変換する。クロック不明時はUINT32_MAXを返す。 */
static inline uint32_t MPC_CyclesToMicroseconds(uint32_t cycles)
{
    return SystemCoreClock != 0U
           ? (uint32_t)(((uint64_t)cycles * 1000000ULL) / SystemCoreClock)
           : UINT32_MAX;
}

#endif /* MPC_TIMING_H */
