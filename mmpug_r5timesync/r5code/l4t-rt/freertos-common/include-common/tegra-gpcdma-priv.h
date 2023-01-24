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

#ifndef _TEGRA_GPCDMA_PRIV_H_
#define _TEGRA_GPCDMA_PRIV_H_

#include <stdbool.h>
#include <stdint.h>

#include <FreeRTOS.h>
#include <semphr.h>

#include <clk-tegra.h>
#include <tegra-gpcdma.h>

/*
 * Both AON and SCE GPCDMA controllers support 8 channels. Hence,
 * setting number of DMA channels as 8.
 */
#define TEGRA_GPCDMA_NUM_CHANNELS 8

struct tegra_gpcdma_id {
	const char *devname;
	uint32_t base_addr;
	const struct tegra_rst *rst;
	const uint32_t irqs[TEGRA_GPCDMA_NUM_CHANNELS];
};

struct tegra_gpcdma_channel {
	SemaphoreHandle_t irq_sem;
	uint32_t chan_base;
	uint32_t irq;
	bool synchronous;
	bool busy;
	dma_callback *callback;
	void *callback_param;
};

struct tegra_gpcdma_ctlr {
	/*
	 * The driver relies on this being the first field, since it casts
	 * pointers between structs tegra_gpcdma_id and tegra_gpcdma_ctlr.
	 */
	struct tegra_gpcdma_id id;
	struct tegra_gpcdma_channel channels[TEGRA_GPCDMA_NUM_CHANNELS];
};

#endif
