/*
 * Copyright (c) 2018 NVIDIA CORPORATION. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of NVIDIA CORPORATION nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _SPE_CLK_H_
#define _SPE_CLK_H_

#define AON_CPU_MAX_RATE 408000
#define AON_APB_MAX_RATE 200000

#define CLK_SRC_PLLAON 0
#define CLK_SRC_PLLP 1
#define CLK_SRC_OSC_UNDIV 2
#define CLK_SRC_CLK_S 3
#define CLK_SRC_MAX 4

#define CLK_ID_AON_CPU_NIC_ACTIVE 0
#define CLK_ID_AON_CPU_NIC_ACTIVE_IRQFIQ 1
#define CLK_ID_AON_CPU_NIC_IDLE_SHALLOW 2
#define CLK_ID_AON_CPU_NIC_IDLE_DEEP 3
#define CLK_ID_AON_CPU_NIC_STBY_SHALLOW 4
#define CLK_ID_AON_CPU_NIC_STBY_DEEP 5
#define CLK_ID_AON_CPU_NIC_DORMANT_SHALLOW 6
#define CLK_ID_AON_CPU_NIC_DORMANT_DEEP 7
#define CLK_ID_AON_APB_ACTIVE 8
#define CLK_ID_AON_APB_ACTIVE_IRQFIQ 9
#define CLK_ID_AON_APB_IDLE_SHALLOW 10
#define CLK_ID_AON_APB_IDLE_DEEP 11
#define CLK_ID_AON_APB_STBY_SHALLOW 12
#define CLK_ID_AON_APB_STBY_DEEP 13
#define CLK_ID_AON_APB_DORMANT_SHALLOW 14
#define CLK_ID_AON_APB_DORMANT_DEEP 15
#define CLK_ID_AON_CPU_NIC 16
#define CLK_ID_AON_APB 17
#define CLK_ID_MAX 18

void spe_clk_init(void);
int spe_clk_set_clk_freq(int clk_id, int clk_src, uint32_t freq_khz);
int spe_clk_set_clk_no_div(int clk_id, int clk_src);
void spe_clk_trigger_switch_fsm(void);

void disable_pllaon(void);
void enable_pllaon(void);

#endif
