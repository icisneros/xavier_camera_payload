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

#include <nvrm_drf.h>

#include <armv7-mmu.h>
#include <cache.h>

static uint32_t pt[4096] __attribute__((aligned(16384)));
static bool armv7_mmu_pxn_supported;

void armv7_mmu_init(void)
{
	uint32_t id_mmfr0, vmsa_support;

	/* If we didn't allocate pt[] statically, we'd do it here instead. */

	__asm__ __volatile__("mrc p15, 0, %0, c0, c1, 4\n\t" : "=r" (id_mmfr0));
	vmsa_support = id_mmfr0 & 0xf;
	armv7_mmu_pxn_supported = vmsa_support >= 4;
}

void armv7_mmu_global_section_configure(uint32_t section_num,
	uint32_t base_pa, uint32_t type_bits, uint32_t ap, bool xn)
{
	uint32_t val;

	val = base_pa | type_bits | ap |
		(xn << ARMV7_MMU_SHORT_XN_SHIFT) |
		ARMV7_MMU_SHORT_TYPE_SECTION;
	if (armv7_mmu_pxn_supported)
		val |= xn << ARMV7_MMU_SHORT_PXN_SHIFT;
	pt[section_num] = val;
}

void armv7_mmu_enable(void)
{
	uint32_t sctlr;

	cache_clean(pt, sizeof(pt));

	/* Set TTBCR = 0 */
	__asm__ __volatile__("mcr p15, 0, %0, c2, c0, 2\n\t" : : "r" (0));

	/* Point TTBR0 at the page table */
	__asm__ __volatile__("mcr p15, 0, %0, c2, c0, 0\n\t" : : "r" (pt));

	/* Configure DACR; Domain Access Control Register as all "client" */
	__asm__ __volatile__("mcr p15, 0, %0, c3, c0, 0\n\t"
		: : "r" (0x55555555));

	/* Invalidate entire unified TLB; TLBIALL */
	__asm__ __volatile__("mcr p15, 0, %0, c8, c7, 0\n\t" : : "r" (0));

	/* Ensure all setup is complete before proceeding */
	__asm__ __volatile__("isb\n\t");

	/* Enable the MMU */
	__asm__ __volatile__("mrc p15, 0, %0, c1, c0, 0\n\t" : "=r" (sctlr));
	sctlr |= BIT(0); // M bit; enable MMU
	__asm__ __volatile__("mcr p15, 0, %0, c1, c0, 0\n\t" : : "r" (sctlr));

	/* Ensure the MMU is on before proceeding */
	__asm__ __volatile__("isb\n\t" : : : "memory");
}
