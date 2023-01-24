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

static inline void dcache_invalidate_all(void)
{
	__asm__ __volatile__(
		"mcr p15, 0, r0, c15, c5, 0\n\t" /* Invalidate entire data cache */
		: : : "memory"
	);
}

#include "cache-aarch32.c"

/* Sequence Reference 8.5.5 of Cortex R5 TRM (Disabling or enabling error checking) */
void cache_enable_ecc(void)
{
	uint32_t actlr;

	cache_disable();

	__asm__ __volatile__(
		"mrc	p15, 0, %0, c1, c0, 1\n\t" /* Read Auxiliary Control Register configuration data */
		"bic	%0, %0, #0x7 << 3\n\t"     /* Clear the field to zero first */
		"orr	%0, %0, #0x5 << 3\n\t"     /* Do not generate abort on ECC errors, enable HW recovery */
		/* WAR for ARM R5 Erratum 780125 */
		/* See http://nvbugs/200322274 */
		/* Disable write burst in the AXI master (DBWR: bit 14) */
		"orr    %0, %0, #0x1 << 14\n\t"
		"mcr	p15, 0, %0, c1, c0, 1\n\t" /* Write Auxiliary Control Register configuration data */
		"dsb\n\t"
		: "=r" (actlr) : : "memory"
	);

	cache_enable();
}
