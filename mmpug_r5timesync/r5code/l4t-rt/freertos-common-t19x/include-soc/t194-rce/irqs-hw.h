/*
 * Copyright (c) 2016, NVIDIA CORPORATION. All rights reserved.
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

#ifndef INCLUDED_T194_RCE_IRQS_HW_H
#define INCLUDED_T194_RCE_IRQS_HW_H

enum {
	/* VIC0 interrupts */
	NV_RCE_IRQ_WDTFIQ,
	NV_RCE_IRQ_WDTIRQ,
	NV_RCE_IRQ_TIMER0,
	NV_RCE_IRQ_TIMER1,
	NV_RCE_IRQ_TIMER2,
	NV_RCE_IRQ_TIMER3,
	NV_RCE_IRQ_MBOX,
	NV_RCE_IRQ_GTE,
	NV_RCE_IRQ_PMU,
	NV_RCE_IRQ_DMA0,
	NV_RCE_IRQ_DMA1,
	NV_RCE_IRQ_DMA2,
	NV_RCE_IRQ_DMA3,
	NV_RCE_IRQ_DMA4,
	NV_RCE_IRQ_DMA5,
	NV_RCE_IRQ_DMA6,
	NV_RCE_IRQ_DMA7,
	NV_RCE_IRQ_LIC0,
	NV_RCE_IRQ_LIC1,
	NV_RCE_IRQ_LIC2,
	NV_RCE_IRQ_LIC3,
	NV_RCE_IRQ_V0RSVD21,
	NV_RCE_IRQ_V0RSVD22,
	NV_RCE_IRQ_VI_HP,
	NV_RCE_IRQ_VI_LP,
	NV_RCE_IRQ_HSM_HP,
	NV_RCE_IRQ_HSM_LP,
	NV_RCE_IRQ_CPU_ERR,
	NV_RCE_IRQ_SC_SYSINTR,
	NV_RCE_IRQ_SC_SYSINTL,
	NV_RCE_IRQ_SC_WARN,
	NV_RCE_IRQ_SC_INFO,

	/* VIC1 interrupts */
	NV_RCE_IRQ_APBERR,
	NV_RCE_IRQ_ACTMON,
	NV_RCE_IRQ_FPUINT,
	NV_RCE_IRQ_PM,
	NV_RCE_IRQ_MC_SBE,
	NV_RCE_IRQ_APBSEC,
	NV_RCE_IRQ_CAR,
	NV_RCE_IRQ_V1RSVD7,
	NV_RCE_IRQ_V1RSVD8,
	NV_RCE_IRQ_V1RSVD9,
	NV_RCE_IRQ_V1RSVD10,
	NV_RCE_IRQ_V1RSVD11,
	NV_RCE_IRQ_V1RSVD12,
	NV_RCE_IRQ_V1RSVD13,
	NV_RCE_IRQ_TOP0_HSP_DB,
	NV_RCE_IRQ_V1RSVD15,
	NV_RCE_IRQ_V1RSVD16,
	NV_RCE_IRQ_CTIIRQ,
	NV_RCE_IRQ_V1RSVD18,
	NV_RCE_IRQ_I2C1,
	NV_RCE_IRQ_I2C3,
	NV_RCE_IRQ_I2C8,
	NV_RCE_IRQ_V1RSVD22,
	NV_RCE_IRQ_V1RSVD23,
	NV_RCE_IRQ_V1RSVD24,
	NV_RCE_IRQ_V1RSVD25,
	NV_RCE_IRQ_V1RSVD26,
	NV_RCE_IRQ_V1RSVD27,
	NV_RCE_IRQ_V1RSVD28,
	NV_RCE_IRQ_V1RSVD29,
	NV_RCE_IRQ_V1RSVD30,
	NV_RCE_IRQ_V1RSVD31,
};

#endif /* INCLUDED_IRQS_HW_H */
