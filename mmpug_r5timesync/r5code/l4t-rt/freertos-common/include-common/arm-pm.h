/*
 * Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef INCLUDE_ARM_PERF_MONITOR_H
#define INCLUDE_ARM_PERF_MONITOR_H

#include <stddef.h>

/* PMCR (Performance Monitor Control Register) */
#define ARM_PMCR_N_OFFSET		11U
#define ARM_PMCR_N_MASK			(0x1fU << ARM_PMCR_N_OFFSET)
#define ARM_PMCR_D_OFFSET		3U
#define ARM_PMCR_D_MASK			(1U << ARM_PMCR_D_OFFSET)
#define ARM_PMCR_C_OFFSET		2U
#define ARM_PMCR_C_MASK			(1U << ARM_PMCR_C_OFFSET)
#define ARM_PMCR_P_OFFSET		1U
#define ARM_PMCR_P_MASK			(1U << ARM_PMCR_P_OFFSET)
#define ARM_PMCR_E_OFFSET		0U
#define ARM_PMCR_E_MASK			(1U << ARM_PMCR_E_OFFSET)

/* PMCNTENSET (Count Enable Set Register) */
#define ARM_PMCNTENSET_C_OFFSET		31U
#define ARM_PMCNTENSET_C_MASK		(1U << ARM_PMCNTENSET_C_OFFSET)

/*
 * Register I/O
 */

#ifdef __ARM_ARCH

#define ARM_PM_REG_OP(regname, id) \
	static inline uint32_t arm_pm_ ## regname ## _get(void) \
	{ \
		uint32_t value; \
		__asm__ __volatile__ ("mrc p15, 0, %0, c9, " id : "=r"(value)); \
		return value; \
	} \
	static inline void arm_pm_ ## regname ## _set(uint32_t value) \
	{ \
		__asm__ __volatile__("mcr p15, 0, %0, c9, " id : : "r"(value)); \
	}

#else /* __ARM_ARCH */

#define ARM_PM_REG_OP(regname, id) \
	static inline uint32_t arm_pm_ ## regname ## _get(void) { return 0; } \
	static inline void arm_pm_ ## regname ## _set(uint32_t value) {}

#endif /* __ARM_ARCH */

/* PMCR (Performance Monitor Control Register) */
ARM_PM_REG_OP(pmcr, "c12, 0")
/* PMCNTENSET (Count Enable Set Register) */
ARM_PM_REG_OP(pmcntenset, "c12, 1")
/* PMCNTENCLR (Count enable Clear Register) */
ARM_PM_REG_OP(pmcntenclr, "c12, 2")
/* PMOVSR (Overflow Flag Status Register) */
ARM_PM_REG_OP(pmovsr, "c12, 3")
/* PMSWINC (Software Increment Register) */
ARM_PM_REG_OP(pmswinc, "c12, 4")
/* PMSELR (Performance Counter Selection Register) */
ARM_PM_REG_OP(pmselr, "c12, 5")
/* PMCCNTR (Cycle Count Register) */
ARM_PM_REG_OP(pmccntr, "c13, 0")
/* PMXEVTYPER (Event Type Selection Register) */
ARM_PM_REG_OP(pmxevtyper, "c13, 1")
/* PMXEVCNTR (Event Count Register) */
ARM_PM_REG_OP(pmxevcntr, "c13, 2")

/* PMCR */
static inline uint32_t arm_pm_get_num_of_counters(void)
{
	return (arm_pm_pmcr_get() & ARM_PMCR_N_MASK) >> ARM_PMCR_N_OFFSET;
}

#endif /* INCLUDE_ARM_PERF_MONITOR_H */
