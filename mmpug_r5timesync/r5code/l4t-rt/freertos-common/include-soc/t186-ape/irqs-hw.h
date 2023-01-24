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

#ifndef _IRQS_HW_H_
#define _IRQS_HW_H_

/*
 * APE has two GICs; the "local" GIC that directly drives the CPU's IRQ/FIQ
 * signals, and the AGIC whose IRQ/FIQ outputs feed into PPIs in the local
 * GIC. This file defines both sets of IRQs
 *
 * The local GIC only has PPIs; no SPIs.
 *
 * First 32 interrupts from AGIC are unused CPU-specific interrupts,
 * we do not use their numbers.
 *
 * The AGIC only has SPIs connected.
 */

/* Global IRQ numbering space base IRQ numbers for the various controllers */
#define NV_APE_LGIC_INTERRUPT_BASE	0

#define NV_APE_AGIC_INTERRUPT_BASE	32

/*
 * Local GIC IRQs
 *
 * These are all PPIs. We use the names and values from aradsp_periph.h.
 * That file defines IRQ numbers that represent "GIC-wide" IRQ numbers, i.e.
 * already include ARM_GIC_IRQ_BASE_PPI within the value, hence why that
 * macro is not included in calculations here.
 *
 * Once aradap_periph.h is available in the SW tree, we will simply include
 * it instead of redefining all these values.
 */
#define NV_APE_ADSP_PERIPH_INTR_ID_GLOBAL_TIMER		27
#define NV_APE_ADSP_PERIPH_INTR_ID_FIQ			28
#define NV_APE_ADSP_PERIPH_INTR_ID_PRIVATE_TIMER	29
#define NV_APE_ADSP_PERIPH_INTR_ID_WATCHDOG_TIMER	30
#define NV_APE_ADSP_PERIPH_INTR_ID_IRQ			31

#include <aragic.h>

#endif
