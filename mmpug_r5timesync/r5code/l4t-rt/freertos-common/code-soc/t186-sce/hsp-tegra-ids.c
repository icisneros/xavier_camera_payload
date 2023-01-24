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

#include <address_map_new.h>

#include <hsp-tegra-hw.h>
#include <hsp-tegra-priv.h>
#include <irqs-hw.h>

struct tegra_hsp_ctx tegra_hsp_ctx_top0 = {
	.id = {
		.devname = "hsp",
		.base_addr = NV_ADDRESS_MAP_TOP0_HSP_BASE,
		.host = TEGRA_HSP_DB_SCE,
		.db_irq = NV_SCE_INTERRUPT_TOP0_HSP_DB + NV_SCE_INTERRUPT_VIC1_BASE,
		.sh_irqs = { -1, -1, -1, -1, -1, -1, -1, -1, },
	},
};
extern const struct tegra_hsp_id tegra_hsp_id_top0 __attribute__((alias("tegra_hsp_ctx_top0")));

struct tegra_hsp_ctx tegra_hsp_ctx_sce = {
	.id = {
		.devname = "sce-hsp",
		.base_addr = NV_ADDRESS_MAP_SCE_HSP_BASE,
		.db_irq = -1,
		.sh_irqs = {
			NV_SCE_INTERRUPT_VIC0_BASE + NV_SCE_INTERRUPT_MBOX,
			/* 1-4 are connected to LIC: */ -1, -1, -1, -1,
			-1, -1, -1,
		},
	},
};
extern const struct tegra_hsp_id tegra_hsp_id_sce __attribute__((alias("tegra_hsp_ctx_sce")));

struct tegra_hsp_ctx tegra_hsp_ctx_bpmp = {
	.id = {
		.devname = "bpmp-hsp",
		.base_addr = NV_ADDRESS_MAP_BPMP_HSP_BASE,
		.db_irq = -1,
		.sh_irqs = { -1, -1, -1, -1, -1, -1, -1, -1, },
	},
};
extern const struct tegra_hsp_id tegra_hsp_id_bpmp __attribute__((alias("tegra_hsp_ctx_bpmp")));

struct tegra_hsp_ctx tegra_hsp_ctx_aon = {
	.id = {
		.devname = "aon-hsp",
		.base_addr = NV_ADDRESS_MAP_AON_HSP_BASE,
		.db_irq = -1,
		.sh_irqs = { -1, -1, -1, -1, -1, -1, -1, -1, },
	},
};
extern const struct tegra_hsp_id tegra_hsp_id_aon __attribute__((alias("tegra_hsp_ctx_aon")));
