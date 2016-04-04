/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#ifndef _CHCONF_V3_H_
#define _CHCONF_V3_H_

#define CYC_CNT (*(volatile uint32_t *)0xF8001020) /* TTC0_2 Counter */

#ifndef _FROM_ASM_
typedef uint16_t cyc_cnt_t;
#endif

/**
 * @brief   Threads descriptor structure extension.
 * @details User fields added to the end of the @p thread_t structure.
 */
#define CH_CFG_THREAD_EXTRA_FIELDS                                          \
  /* Add threads custom fields here.*/                                      \
  uint64_t p_ctime;                                                         \
  cyc_cnt_t p_cref;

/**
 * @brief   Threads initialization hook.
 * @details User initialization code added to the @p chThdInit() API.
 *
 * @note    It is invoked from within @p chThdInit() and implicitly from all
 *          the threads creation APIs.
 */
#define CH_CFG_THREAD_INIT_HOOK(tp) {                                       \
  /* Add threads initialization code here.*/                                \
  /* CPU cycle measurement fields, */                                       \
  /* see http://sourceforge.net/p/chibios/feature-requests/23/ .*/          \
  tp->p_ctime = 0;                                                          \
  tp->p_cref = CYC_CNT;                                                     \
}

/**
 * @brief   Context switch hook.
 * @details This hook is invoked just before switching between threads.
 */
#ifndef _FROM_ASM_
extern uint64_t g_ctime;
#endif

#define CH_CFG_CONTEXT_SWITCH_HOOK(ntp, otp) {                              \
  ntp->p_cref = CYC_CNT;                                                    \
  if (otp) {                                                                \
    cyc_cnt_t cnt = ntp->p_cref - otp->p_cref;                               \
    otp->p_ctime += cnt;                                                    \
    g_ctime += cnt;                                                         \
  }                                                                         \
}

/**
 * @brief   WFI configuration
 */
#define ARM_ENABLE_WFI_IDLE     TRUE
#define ARM_WFI_IMPL            asm volatile ("wfi")

/**
 * @brief   FPU configuration
 */
#define ARM_FPU                 neon


#endif

