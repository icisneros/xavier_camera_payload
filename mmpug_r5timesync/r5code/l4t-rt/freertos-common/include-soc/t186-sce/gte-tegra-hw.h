/*
 * Copyright (c) 2015, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _GTE_TEGRA_HW_H_
#define _GTE_TEGRA_HW_H_

extern struct tegra_gte_id tegra_gte_id_sce;

//GTE Interrupt connections. For slice 0
#define NV_SCE_GTE_SLICE0_IRQ_WDTFIQ		0
#define NV_SCE_GTE_SLICE0_IRQ_WDTIRQ		1
#define NV_SCE_GTE_SLICE0_IRQ_TIMER0		2
#define NV_SCE_GTE_SLICE0_IRQ_TIMER1		3
#define NV_SCE_GTE_SLICE0_IRQ_TIMER2		4
#define NV_SCE_GTE_SLICE0_IRQ_TIMER3		5
#define NV_SCE_GTE_SLICE0_IRQ_MBOX		6
#define NV_SCE_GTE_SLICE0_IRQ_GTE		7
#define NV_SCE_GTE_SLICE0_IRQ_PMU		8
#define NV_SCE_GTE_SLICE0_IRQ_DMA0		9
#define NV_SCE_GTE_SLICE0_IRQ_DMA1		10
#define NV_SCE_GTE_SLICE0_IRQ_DMA2		11
#define NV_SCE_GTE_SLICE0_IRQ_DMA3		12
#define NV_SCE_GTE_SLICE0_IRQ_DMA4		13
#define NV_SCE_GTE_SLICE0_IRQ_DMA5		14
#define NV_SCE_GTE_SLICE0_IRQ_DMA6		15
#define NV_SCE_GTE_SLICE0_IRQ_DMA7		16
#define NV_SCE_GTE_SLICE0_IRQ_LIC0		17
#define NV_SCE_GTE_SLICE0_IRQ_LIC1		18
#define NV_SCE_GTE_SLICE0_IRQ_LIC2		19
#define NV_SCE_GTE_SLICE0_IRQ_LIC3		20
#define NV_SCE_GTE_SLICE0_IRQ_V0RSVD21		21
#define NV_SCE_GTE_SLICE0_IRQ_V0RSVD22		22
#define NV_SCE_GTE_SLICE0_IRQ_VI_HP		23
#define NV_SCE_GTE_SLICE0_IRQ_VI_LP		24
#define NV_SCE_GTE_SLICE0_IRQ_HSM_HP		25
#define NV_SCE_GTE_SLICE0_IRQ_HSM_LP		26
#define NV_SCE_GTE_SLICE0_IRQ_CPU_ERR		27
#define NV_SCE_GTE_SLICE0_IRQ_SC_SYSINTR	28
#define NV_SCE_GTE_SLICE0_IRQ_SC_SYSINTL	29
#define NV_SCE_GTE_SLICE0_IRQ_SC_WARN		30
#define NV_SCE_GTE_SLICE0_IRQ_SC_INFO		31

//GTE Interrupt connections. For slice 1
#define NV_SCE_GTE_SLICE1_IRQ_APBERR		0
#define NV_SCE_GTE_SLICE1_IRQ_ACTMON		1
#define NV_SCE_GTE_SLICE1_IRQ_FPUINT		2
#define NV_SCE_GTE_SLICE1_IRQ_PM		3
#define NV_SCE_GTE_SLICE1_IRQ_MC_SBE		4
#define NV_SCE_GTE_SLICE1_IRQ_APBSEC		5
#define NV_SCE_GTE_SLICE1_IRQ_CAR		6
#define NV_SCE_GTE_SLICE1_IRQ_V1RSVD7		7
#define NV_SCE_GTE_SLICE1_IRQ_V1RSVD8		8
#define NV_SCE_GTE_SLICE1_IRQ_V1RSVD9		9
#define NV_SCE_GTE_SLICE1_IRQ_V1RSVD10		10
#define NV_SCE_GTE_SLICE1_IRQ_V1RSVD11		11
#define NV_SCE_GTE_SLICE1_IRQ_V1RSVD12		12
#define NV_SCE_GTE_SLICE1_IRQ_V1RSVD13		13
#define NV_SCE_GTE_SLICE1_IRQ_TOP0_HSP_DB	14
#define NV_SCE_GTE_SLICE1_IRQ_V1RSVD15		15
#define NV_SCE_GTE_SLICE1_IRQ_V1RSVD16		16
#define NV_SCE_GTE_SLICE1_IRQ_CTIIRQ		17
#define NV_SCE_GTE_SLICE1_IRQ_V1RSVD18		18
#define NV_SCE_GTE_SLICE1_IRQ_I2C1		19
#define NV_SCE_GTE_SLICE1_IRQ_I2C3		20
#define NV_SCE_GTE_SLICE1_IRQ_I2C8		21
#define NV_SCE_GTE_SLICE1_IRQ_V1RSVD22		22
#define NV_SCE_GTE_SLICE1_IRQ_V1RSVD23		23
#define NV_SCE_GTE_SLICE1_IRQ_V1RSVD24		24
#define NV_SCE_GTE_SLICE1_IRQ_V1RSVD25		25
#define NV_SCE_GTE_SLICE1_IRQ_V1RSVD26		26
#define NV_SCE_GTE_SLICE1_IRQ_V1RSVD27		27
#define NV_SCE_GTE_SLICE1_IRQ_V1RSVD28		28
#define NV_SCE_GTE_SLICE1_IRQ_V1RSVD29		29
#define NV_SCE_GTE_SLICE1_IRQ_V1RSVD30		30
#define NV_SCE_GTE_SLICE1_IRQ_V1RSVD31		31

#endif
