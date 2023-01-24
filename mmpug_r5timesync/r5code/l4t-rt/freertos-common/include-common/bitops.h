/*
 * Copyright (c) 2016-2017, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef INCLUDE_FREERTOS_COMMON_BITOPS_H
#define INCLUDE_FREERTOS_COMMON_BITOPS_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

static inline uint32_t bit_number(uint32_t value)
{
#if __GNUC__
	return __builtin_ctz(value);
#else
	uint32_t n = 0;

	if (value & 0xffff0000U)
		n += 16U;
	if (value & 0xff00ff00U)
		n += 8U;
	if (value & 0xf0f0f0f0U)
		n += 4U;
	if (value & 0xccccccccU)
		n += 2U;
	if (value & 0xaaaaaaaaU)
		n += 1U;

	return n;
#endif
}

static inline uint32_t next_bit(
	uint32_t firstbit,
	uint32_t nbits,
	const uint32_t words[],
	uint32_t invert)
{
	uint32_t word;
	uint32_t lsb_only;

	for (;;) {
		if (firstbit >= nbits)
			return nbits;

		word = words[firstbit / 32U] ^ invert;

		/* Mask before firstbit */
		word &= ~((1U << (firstbit % 32U)) - 1U);

		if (word != 0)
			break;

		firstbit = (firstbit + 32U) & ~31U;
	}

	lsb_only = word & ~(word - 1U);

	firstbit = (firstbit & ~31U) + bit_number(lsb_only);

	if (firstbit >= nbits)
		return nbits;

	return firstbit;
}

static inline uint32_t next_one(
	uint32_t firstbit,
	uint32_t nbits,
	const uint32_t words[])
{
	return next_bit(firstbit, nbits, words, 0U);
}

static inline uint32_t next_zero(
	uint32_t firstbit,
	uint32_t nbits,
	const uint32_t words[])
{
	return next_bit(firstbit, nbits, words, ~0U);
}

static inline uint64_t hilo_to_64(
	uint32_t hi32,
	uint32_t lo32)
{
#if defined(__BYTE_ORDER__)
	union {
		struct {
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
			uint32_t lo32, hi32;
#else
			uint32_t hi32, lo32;
#endif
		} value32;
		uint64_t value64;
	} value = { .value32 = { .hi32 = hi32, .lo32 = lo32 }};

	return value.value64;
#else
	/* GCC spills registers if this is used */
   	return ((uint64_t)hi32 << 32) | lo32;
#endif
}

#endif /* INCLUDE_FREERTOS_COMMON_BITOPS_H */
