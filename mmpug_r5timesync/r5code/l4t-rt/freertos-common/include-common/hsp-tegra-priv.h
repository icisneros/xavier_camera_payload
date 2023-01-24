/*
 * Copyright (c) 2015-2016 NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _HSP_TEGRA_PRIV_H_
#define _HSP_TEGRA_PRIV_H_

#include <stdint.h>

#include <hsp-tegra.h>

#define MAX_TEGRA_HSP_SM  8U

struct tegra_hsp_id {
	const char *devname;
	uint32_t base_addr;
	uint32_t host;
	int32_t db_irq;
	int32_t sh_irqs[MAX_TEGRA_HSP_SM];
};

struct tegra_hsp_ctx {
	/*
	 * The driver relies on this being the first field, since it casts
	 * pointers between structs tegra_hsp_id and tegra_hsp_ctx.
	 */
	const struct tegra_hsp_id id;
	tegra_hsp_callback db_callback;
	uint32_t db_base;
	struct {
		void (*callback)(void *, uint32_t);
		void *opaque;
	} sm[MAX_TEGRA_HSP_SM];
	uint8_t cbtype[MAX_TEGRA_HSP_SM];
	uint8_t n_sm;
	uint8_t n_ss;
	uint8_t n_as;
	uint8_t n_db;
	uint8_t n_si;
};

#endif
