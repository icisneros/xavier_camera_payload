/*
 * Copyright (c) 2015-2016, NVIDIA CORPORATION.  All rights reserved.
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

#include <stdint.h>

#include <barriers.h>
#include <cache.h>
#include <cache-hw.h>
#include <cache-l2.h>
#include <err-hook.h>

/*
 * Note: This file should be included from a top-level CPU-specific file,
 * rather than compiled directly.
 *
 * Files that include this file should:
 * #define CACHE_LINE SIZE (in bytes)
 * Implement function void dcache_invalidate_all(void)
 */

#define CACHE_LINE_MASK (~(CACHE_LINE_SIZE - 1))

/* Not all systems have separate L2. Do nothing if no L2 driver is present */
void __attribute__((weak)) cache_l2_disable(void)
{
}

void __attribute__((weak)) cache_l2_enable(void)
{
}

void __attribute__((weak)) cache_l2_invalidate(void *base, size_t length)
{
}

void __attribute__((weak)) cache_l2_clean(const void *base, size_t length)
{
}

static inline void dcache_clean_all(void)
{
	uint32_t clidr;
	uint32_t levels, level;

	/* Read CLIDR */
	__asm__ __volatile__(
		"mrc p15, 1, %0, c0, c0, 1\n\t"
		: "=r" (clidr)
	);
	levels = (clidr >> 24) & 7;

	for (level = 0; level < levels; level++) {
		uint32_t type;
		uint32_t ccsidr;
		uint32_t log2_line_size_words;
		uint32_t log2_line_size_bytes;
		uint32_t max_way;
		int way_shift; /* signed due to clz prototype */
		int way, set; /* signed for loop test */

		type = (clidr >> (3 * level)) & 7;
		if (type < 2)
			continue;

		/* Write CSSELR (Cache Size SELection Register) */
		__asm__ __volatile__(
			"mcr p15, 2, %0, c0, c0, 0\n\t"
			"isb\n\t"
			: : "r" (level * 2)
		);
		/* Read CCSIDR (CaChe Size ID Register) */
		__asm__ __volatile__(
			"mrc p15, 1, %0, c0, c0, 0\n\t"
			: "=r" (ccsidr)
		);

		log2_line_size_words = (ccsidr & 7) + 2;
		/* words->bytes: *4 => +log2(4) in log2 */
		log2_line_size_bytes = log2_line_size_words + 2;

		max_way = (ccsidr >> 3) & 0x3ff;
		way_shift = __builtin_clz(max_way);

		set = (ccsidr >> 13) & 0x7fff;

		do {
			way = max_way;
			do {
				uint32_t dccsw_param =
					(way << way_shift) |
					(set << log2_line_size_bytes) |
					(level << 1);
				__asm__ __volatile__(
					"mcr p15, 0, %0, c7, c10, 2\n\t"
					: : "r" (dccsw_param) : "memory"
				);
				way--;
			} while (way >= 0);
			set--;
		} while (set >= 0);
	}
	barrier_cache_op_complete();
}

static inline void icache_invalidate_all(void)
{
	__asm__ __volatile__(
		"mcr	p15, 0, %0, c7, c5, 0\n\t" /* ICIALLU */
		: : "r" (0) : "memory"
	);
}

static inline void branch_predictor_invalidate_all(void)
{
	__asm__ __volatile__(
		"mcr	p15, 0, %0, c7, c5, 6\n\t" /* BPIALL */
		: : "r" (0) : "memory"
	);
}

void cache_disable(void)
{
	__asm__ __volatile__(
		"mrc	p15, 0, r0, c1, c0, 0\n\t" /* Read System Control Register configuration data */
		"bic	r0, r0, #0x1 << 12\n\t" /* instruction cache disable */
		"bic	r0, r0, #0x1 << 2\n\t" /* data cache disable */
		"dsb\n\t"
		"mcr	p15, 0, r0, c1, c0, 0\n\t" /* Write System Control Register configuration data */
		"isb\n\t"
		: : : "r0", "memory"
	);

	cache_l2_disable();
	dcache_clean_all();
}

void cache_enable(void)
{
	uint32_t sctlr;

	__asm__ __volatile__(
		"mrc	p15, 0, %0, c1, c0, 0\n\t" /* Read System Control Register configuration data */
		"orr	%0, %0, #0x1 << 12\n\t" /* instruction cache enable */
		"orr	%0, %0, #0x1 << 2\n\t" /* data cache enable */
		"dsb\n\t"
		: "=r" (sctlr) : : "memory"
	);
	dcache_invalidate_all();
	icache_invalidate_all();
	branch_predictor_invalidate_all();
	__asm__ __volatile__(
		"mcr	p15, 0, %0, c1, c0, 0\n\t" /* enabled cache RAMs */
		"isb\n\t"
		: : "r" (sctlr) : "memory"
	);

	cache_l2_enable();
}

void cache_invalidate(void *base, size_t length)
{
	uint32_t base_addr = (uint32_t)base;
	size_t len;

	length += (base_addr & ~CACHE_LINE_MASK);
	base_addr &= CACHE_LINE_MASK;
	base = (void *)base_addr;

	length += CACHE_LINE_SIZE - 1;
	length &= CACHE_LINE_MASK;

	for (len = 0; len < length; len += CACHE_LINE_SIZE) {
		/* DCIMVAC; DCache Invalidate by MVA to PoC */
		__asm__ __volatile__(
			"mcr p15, 0, %0, c7, c6, 1\n\t"
			: : "r" (base_addr) : "memory"
		);
		base_addr += CACHE_LINE_SIZE;
	}
	barrier_cache_op_complete();

	cache_l2_invalidate(base, length);
}

void cache_clean(const void *base, size_t length)
{
	uint32_t base_addr = (uint32_t)base;
	size_t len;

	length += (base_addr & ~CACHE_LINE_MASK);
	base_addr &= CACHE_LINE_MASK;
	base = (void *)base_addr;

	length += CACHE_LINE_SIZE - 1;
	length &= CACHE_LINE_MASK;

	for (len = 0; len < length; len += CACHE_LINE_SIZE) {
		/* DCCMVAC; DCache Clean by MVA to PoC */
		__asm__ __volatile__(
			"mcr p15, 0, %0, c7, c10, 1\n\t"
			: : "r" (base_addr) : "memory"
		);
		base_addr += CACHE_LINE_SIZE;
	}
	barrier_cache_op_complete();

	cache_l2_clean(base, length);
}
