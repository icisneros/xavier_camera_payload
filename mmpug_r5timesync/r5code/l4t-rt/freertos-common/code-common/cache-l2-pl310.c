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

#include <stdint.h>

#include <cache-l2.h>
#include <cache-hw.h>
#include <reg-access.h>

#define PL310_CONTROL			0x100
#define PL310_CACHE_SYNC		0x730
#define PL310_INVALIDATE_LINE_BY_PA	0x770
#define PL310_INVALIDATE_LINE_BY_WAY	0x77c
#define PL310_CLEAN_LINE_BY_PA		0x7b0

static inline void cache_l2_sync(void)
{
	writel(0, PL310_BASE + PL310_CACHE_SYNC);
	while (readl(PL310_BASE + PL310_CACHE_SYNC) & 1)
		;
}

void cache_l2_enable(void)
{
	writel(0xffff, PL310_BASE + PL310_INVALIDATE_LINE_BY_WAY);
	while (readl(PL310_BASE + PL310_CACHE_SYNC) & 0xffff)
		;
	cache_l2_sync();
}

void cache_l2_invalidate(void *base, size_t length)
{
	uint32_t base_addr = (uint32_t)base;

	while (length) {
		writel(base_addr, PL310_BASE + PL310_INVALIDATE_LINE_BY_PA);
		base_addr += CACHE_LINE_SIZE;
		length -= CACHE_LINE_SIZE;
	}
	cache_l2_sync();
}

void cache_l2_clean(const void *base, size_t length)
{
	uint32_t base_addr = (uint32_t)base;

	while (length) {
		writel(base_addr, PL310_BASE + PL310_CLEAN_LINE_BY_PA);
		base_addr += CACHE_LINE_SIZE;
		length -= CACHE_LINE_SIZE;
	}
	cache_l2_sync();
}
