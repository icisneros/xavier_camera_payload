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

#include <irqs-hw.h>
#include <tegra-adma-priv.h>

struct tegra_adma_ctlr tegra_adma_ctlr_ape = {
	.dma_id = {
		.devname = "adma-ape",
		.base_addr = 0x02930000,
		.irq = {
			NV_APE_AGIC_INTR_ID_ADMA_EOT1,
			NV_APE_AGIC_INTR_ID_ADMA_EOT2,
			NV_APE_AGIC_INTR_ID_ADMA_EOT3,
			NV_APE_AGIC_INTR_ID_ADMA_EOT4,
			NV_APE_AGIC_INTR_ID_ADMA_EOT5,
			NV_APE_AGIC_INTR_ID_ADMA_EOT6,
			NV_APE_AGIC_INTR_ID_ADMA_EOT7,
			NV_APE_AGIC_INTR_ID_ADMA_EOT8,
		},
	},
};
extern struct tegra_adma_ctlr tegra_adma_id_ape __attribute__((alias("tegra_adma_ctlr_ape")));
