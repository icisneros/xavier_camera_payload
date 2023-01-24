/*
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
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

#include <address_map_new.h>

#include <clk-tegra-hw.h>
#include <irqs-hw.h>
#include <tegra-gpcdma-priv.h>

struct tegra_gpcdma_ctlr tegra_gpcdma_ctlr_sce = {
	.id = {
		.devname = "dma-sce",
		.base_addr = NV_ADDRESS_MAP_SCE_DMA_BASE,
		.rst = tegra_rst_sce_gpcdma,
		.irqs = {
			NV_SCE_IRQ_DMA0,
			NV_SCE_IRQ_DMA1,
			NV_SCE_IRQ_DMA2,
			NV_SCE_IRQ_DMA3,
			NV_SCE_IRQ_DMA4,
			NV_SCE_IRQ_DMA5,
			NV_SCE_IRQ_DMA6,
			NV_SCE_IRQ_DMA7,
		},
	},
};
extern struct tegra_gpcdma_id tegra_gpcdma_id_sce __attribute__((alias("tegra_gpcdma_ctlr_sce")));
