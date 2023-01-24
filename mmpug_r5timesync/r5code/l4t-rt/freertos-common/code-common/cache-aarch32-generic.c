/*
 * Copyright (c) 2015-2018, NVIDIA CORPORATION.  All rights reserved.
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

void cache_enable_ecc(void)
{
}

static inline void dcache_invalidate_all(void)
{
	uint32_t clidr;
	uint32_t levels, level;

	/* Read CLIDR */
	__asm__ __volatile__(
		"mrc p15, 1, %0, c0, c0, 1\n\t"
		: "=r" (clidr) : :
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
			: : "r" (level * 2) :
		);
		/* Read CCSIDR (CaChe Size ID Register) */
		__asm__ __volatile__(
			"mrc p15, 1, %0, c0, c0, 0\n\t"
			: "=r" (ccsidr) : :
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
				uint32_t dcisw_param =
					(way << way_shift) |
					(set << log2_line_size_bytes) |
					(level << 1);
				__asm__ __volatile__(
					"mcr p15, 0, %0, c7, c6, 2\n\t"
					: : "r" (dcisw_param) : "memory"
				);
				way--;
			} while (way >= 0);
			set--;
		} while (set >= 0);
	}
	barrier_cache_op_complete();
}

#include "cache-aarch32.c"
