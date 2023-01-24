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
 * This driver provides a simplified programming interface to an ARMv7 VMSA
 * MMU. Currently, only sections are supported; no supersections or large
 * pages. Consequently, only a single level of page table is required. Only
 * TTBR0 is supported. Only the short descriptor format is supported. All
 * accesses are assumed to be secure.
 */
#ifndef _ARMV7_MPU_H_
#define _ARMV7_MPU_H_

#include <stdbool.h>
#include <stdint.h>

#include <macros.h>

#define ARMV7_MMU_SECTION_SHIFT		20
#define ARMV7_MMU_SECTION_SIZE		BIT(ARMV7_MMU_SECTION_SHIFT)

#define ARMV7_MMU_SHORT_PA_SHIFT	20
#define ARMV7_MMU_SHORT_S_SHIFT		16
#define ARMV7_MMU_SHORT_AP2_SHIFT	15
#define ARMV7_MMU_SHORT_TEX_SHIFT	12
#define ARMV7_MMU_SHORT_AP10_SHIFT	10
#define ARMV7_MMU_SHORT_XN_SHIFT	4
#define ARMV7_MMU_SHORT_C_SHIFT		3
#define ARMV7_MMU_SHORT_B_SHIFT		2
#define ARMV7_MMU_SHORT_PXN_SHIFT	0

#define ARMV7_MMU_SHORT_TYPE_SECTION	2

#define ARMV7_MMU_SHORT_AP_NONE		((0 << ARMV7_MMU_SHORT_AP2_SHIFT) | \
					 (0 << ARMV7_MMU_SHORT_AP10_SHIFT))
#define ARMV7_MMU_SHORT_AP_RW		((0 << ARMV7_MMU_SHORT_AP2_SHIFT) | \
					 (3 << ARMV7_MMU_SHORT_AP10_SHIFT))
#define ARMV7_MMU_SHORT_AP_RO		((4 << ARMV7_MMU_SHORT_AP2_SHIFT) | \
					 (2 << ARMV7_MMU_SHORT_AP10_SHIFT))

/*
 * Memory type constants. There are some other legal combinations,
 * but I hope this subset is all we need.
 */
#define ARMV7_MMU_SHORT_STRONGLY_ORDERED \
	(0 << ARMV7_MMU_SHORT_TEX_SHIFT)
#define ARMV7_MMU_SHORT_SHAREABLE_DEV \
	((0 << ARMV7_MMU_SHORT_TEX_SHIFT) | \
	BIT(ARMV7_MMU_SHORT_B_SHIFT))
#define ARMV7_MMU_SHORT_DEV \
	(2 << ARMV7_MMU_SHORT_TEX_SHIFT)
#define ARMV7_MMU_SHORT_NORMAL_UNCACHED_SHAREABLE \
	(1 << ARMV7_MMU_SHORT_TEX_SHIFT) | \
	BIT(ARMV7_MMU_SHORT_S_SHIFT)
#define ARMV7_MMU_SHORT_NORMAL_UNCACHED \
	(1 << ARMV7_MMU_SHORT_TEX_SHIFT)
#define ARMV7_MMU_SHORT_NORMAL_CACHED_SHAREABLE \
	((1 << ARMV7_MMU_SHORT_TEX_SHIFT) | \
	BIT(ARMV7_MMU_SHORT_S_SHIFT) | \
	BIT(ARMV7_MMU_SHORT_C_SHIFT) | \
	BIT(ARMV7_MMU_SHORT_B_SHIFT))
#define ARMV7_MMU_SHORT_NORMAL_CACHED \
	((1 << ARMV7_MMU_SHORT_TEX_SHIFT) | \
	BIT(ARMV7_MMU_SHORT_C_SHIFT) | \
	BIT(ARMV7_MMU_SHORT_B_SHIFT))

/*
 * Perform any initialization required before configuring the page tables.
 *
 * This must be called before any other MMU-related function.
 */
void armv7_mmu_init(void);

/*
 * Convert an address into an ARM MMU section number. Useful for calling
 * armv7_mmu_global_section_configure().
 */
static inline uint32_t armv7_mmu_section_of_addr(uint32_t addr)
{
	return addr >> ARMV7_MMU_SECTION_SHIFT;
}

/*
 * Convert an ARM MMU section number into a base address.
 */
static inline uint32_t armv7_mmu_addr_of_section(uint32_t section)
{
	return section << ARMV7_MMU_SECTION_SHIFT;
}

/*
 * Program the mapping for one MMU section.
 *
 * This should be called for every section, or at least every section SW
 * will access. This must be called before armv7_mmu_enable().
 *
 * Parameters:
 * section_num: The (1MiB) section number to configure.
 * base_pa:	The (unshifted) physical address this section translates to.
 * type_bits:	One of the "memory type constants" from above.
 * ap:		One of the ARMV7_MMU_SHORT_AP_* constants from above.
 * xn:		If true, disable code execution for this section.
 */
void armv7_mmu_global_section_configure(uint32_t section_num,
	uint32_t base_pa, uint32_t type_bits, uint32_t ap, bool xn);

/*
 * Enable the MMU.
 *
 * Once this is called, the MMU configuration is fixed; other functions such
 * as armv7_mmu_global_section_configure() may no longer be called.
 */
void armv7_mmu_enable(void);

#endif
