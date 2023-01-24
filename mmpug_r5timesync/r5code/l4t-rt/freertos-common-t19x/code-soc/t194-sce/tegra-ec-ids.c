/*
 * Copyright (c) 2018, NVIDIA CORPORATION.  All rights reserved.
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
#include <arsce_hsm_error.h>
#include <arsce_hsm_error_collator.h>
#include <arsce_ec.h>
#include <argpcdma_sce_errcollator.h>
#include <tegra-ec.h>
#include <tegra-ec-priv.h>

struct tegra_ec tegra_ec_sce = {
	.conf = {
		.devname = "sce-ec",
		.base = NV_ADDRESS_MAP_SCE_ERR_COLLATOR_BASE + SCE_EC_REGS_FEATURE_0,
		.fatal_err_index = NV_SCE_HSM_ERROR_SCE_EC_CRITICAL_ERR,
		.corr_err_index = NV_SCE_HSM_ERROR_SCE_EC_CORR_ERR,
		.user_value_support = 1,
		.error_inject_mask[0] = 0x7FF,
		.error_inject_mask[1] = 0x20,
		.error_inject_mask[2] = 0x1C02000,
	},
};
extern const struct tegra_ec_id tegra_ec_id_sce
__attribute__((alias("tegra_ec_sce")));

struct tegra_ec tegra_ec_hsm = {
	.conf = {
		.devname = "hsm-ec",
		.base = NV_ADDRESS_MAP_HSM_BASE + SCE_HSM_ERROR_COLLATOR_FEATURE_0,
		.fatal_err_index = NV_SCE_HSM_ERROR_HSM_CRITICAL_ERR,
		.user_value_support = 0,
		.error_inject_mask[0] = 0xF,
	},
};
extern const struct tegra_ec_id tegra_ec_id_hsm
__attribute__((alias("tegra_ec_hsm")));

struct tegra_ec tegra_ec_scedma = {
	.conf = {
		.devname = "scedma-ec",
		.base = NV_ADDRESS_MAP_SCE_DMA_BASE + GPCDMA_SCE_ERRCOLLATOR_FEATURE_0,
		.fatal_err_index = NV_SCE_HSM_ERROR_SCE_GPCDMA_CRITICAL_ERR,
		.corr_err_index = NV_SCE_HSM_ERROR_SCE_GPCDMA_CORRECTED_ERR,
		.user_value_support = 0,
		.error_inject_mask[0] = 0x1FFF,
	},
};
extern const struct tegra_ec_id tegra_ec_id_scedma
__attribute__((alias("tegra_ec_scedma")));

