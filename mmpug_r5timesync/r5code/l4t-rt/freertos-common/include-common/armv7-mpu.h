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

/*
 * This driver provides a slightly simplified programming interface to an
 * ARMv7 PMSA MPU. Currently, only systems with unified regions are supported,
 * since APIs only exist to configure data regions and not instruction regions.
 */
#ifndef _ARMV7_MPU_H_
#define _ARMV7_MPU_H_

#include <stdint.h>

#include <macros.h>

#define R5_DRACR_XN		BIT(12)

#define R5_DRACR_AP_SHIFT	8
#define R5_DRACR_AP_NONE	(0 << R5_DRACR_AP_SHIFT)
#define R5_DRACR_AP_RW		(3 << R5_DRACR_AP_SHIFT)
#define R5_DRACR_AP_RO		(6 << R5_DRACR_AP_SHIFT)

#define R5_DRACR_TEX_SHIFT	3

#define R5_DRACR_S_SHIFT	2

#define R5_DRACR_C_SHIFT	1

#define R5_DRACR_B_SHIFT	0

/*
 * Region access control constants. There are some other legal combinations,
 * but I hope this subset is all we need.
 */
#define R5_DRACR_STRONGLY_ORDERED		(0 << R5_DRACR_TEX_SHIFT)
#define R5_DRACR_SHAREABLE_DEV			((0 << R5_DRACR_TEX_SHIFT) | \
						BIT(R5_DRACR_B_SHIFT))
#define R5_DRACR_DEV				(2 << R5_DRACR_TEX_SHIFT)
#define R5_DRACR_NORMAL_UNCACHED_SHAREABLE	(1 << R5_DRACR_TEX_SHIFT) | \
						BIT(R5_DRACR_S_SHIFT)
#define R5_DRACR_NORMAL_UNCACHED		(1 << R5_DRACR_TEX_SHIFT)
#define R5_DRACR_NORMAL_CACHED_SHAREABLE	((1 << R5_DRACR_TEX_SHIFT) | \
						BIT(R5_DRACR_S_SHIFT) | \
						BIT(R5_DRACR_C_SHIFT) | \
						BIT(R5_DRACR_B_SHIFT))
#define R5_DRACR_NORMAL_CACHED			((1 << R5_DRACR_TEX_SHIFT) | \
						BIT(R5_DRACR_C_SHIFT) | \
						BIT(R5_DRACR_B_SHIFT))

/*
 * Convert a region size in bytes to the MPU's register encoding of the region
 * size. The supplied number of bytes must have a single bit set, and be at
 * least 32. The intention is for this function call to result in a compile-
 * time constant where possible, hence its presence in the header file.
 */
static inline uint32_t r5mpu_size(unsigned int size_bytes)
{
	return 31 - __builtin_clz(size_bytes - 1);
}

/*
 * Returns the number of MPU data regions supported by the current CPU
 */
int r5mpu_dregion_count(void);

/*
 * Configures an MPU data region, and enables it.
 *
 * Parameters:
 * region:	The region ID to configure/enable.
 * base:	The base address of the region. This must be aligned to match
 * 		the size of the region.
 * size:	The size of the region, as encoded by r5mpu_size().
 * access:	The access configuration for the region. A bitwise or of:
 * 		R5_DRACR_XN, R5_DRACR_AP, the region access control constants above.
 */
void r5mpu_dregion_config_enable(int region, uint32_t base, uint32_t size, uint32_t access);

/*
 * Disables an MPU data region.
 *
 * Parameters:
 * region:	The region ID to configure/enable.
 */
void r5mpu_dregion_disable(int region);

/*
 * Enables the MPU. This must happen after all regions are configured.
 */
void r5mpu_enable(void);

/*
 * Disables the MPU. This restores the MPU's default mapping. Careful
 * attention must be paid to code layout so that execution can successfully
 * continue after this call executes.
 */
void r5mpu_disable(void);

#endif
