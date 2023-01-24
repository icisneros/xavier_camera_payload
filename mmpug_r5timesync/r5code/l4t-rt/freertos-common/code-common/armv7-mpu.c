/*
 * Copyright (c) 2015-2017, NVIDIA CORPORATION.  All rights reserved.
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

#include <armv7-mpu.h>

#define R5_MPUIR_0_DREGION_RANGE	15:8

#define R5_DRSR_0_RSIZE_RANGE	5:1
#define R5_DRSR_0_EN_RANGE	0:0

#define R5_SCTLR_0_BR_RANGE		17:17
#define R5_SCTLR_0_M_RANGE		0:0

int r5mpu_dregion_count(void)
{
	uint32_t mpuir;

	__asm__ __volatile__("mrc p15, 0, %0, c0, c0, 4\n\t" : "=r" (mpuir));

	return NV_DRF_VAL(R5, MPUIR, DREGION, mpuir);;
}

void r5mpu_dregion_config_enable(int region, uint32_t base, uint32_t size,
	uint32_t access)
{
	uint32_t size_r;

	size_r = NV_DRF_NUM(R5, DRSR, RSIZE, size) |
		NV_DRF_NUM(R5, DRSR, EN, 1);

	__asm__ __volatile__("mcr p15, 0, %0, c6, c2, 0\n\t" : : "r" (region));
	__asm__ __volatile__("mcr p15, 0, %0, c6, c1, 0\n\t" : : "r" (base));
	__asm__ __volatile__("mcr p15, 0, %0, c6, c1, 4\n\t" : : "r" (access));
	__asm__ __volatile__("mcr p15, 0, %0, c6, c1, 2\n\t" : : "r" (size_r));
}

void r5mpu_dregion_disable(int region)
{
	__asm__ __volatile__("mcr p15, 0, %0, c6, c2, 0\n\t" : : "r" (region));
	__asm__ __volatile__("mcr p15, 0, %0, c6, c1, 2\n\t" : : "r" (0));
}

void r5mpu_enable(void)
{
	uint32_t sctlr;

	__asm__ __volatile__("mrc p15, 0, %0, c1, c0, 0\n\t" : "=r" (sctlr));
	sctlr = NV_FLD_SET_DRF_NUM(R5, SCTLR, BR, 0, sctlr);
	sctlr = NV_FLD_SET_DRF_NUM(R5, SCTLR, M, 1, sctlr);
	__asm__ __volatile__("mcr p15, 0, %0, c1, c0, 0\n\t" : : "r" (sctlr));
}

void r5mpu_disable(void)
{
	uint32_t sctlr;

	__asm__ __volatile__("mrc p15, 0, %0, c1, c0, 0\n\t" : "=r" (sctlr));
	sctlr = NV_FLD_SET_DRF_NUM(R5, SCTLR, M, 0, sctlr);
	__asm__ __volatile__("mcr p15, 0, %0, c1, c0, 0\n\t" : : "r" (sctlr));
}
