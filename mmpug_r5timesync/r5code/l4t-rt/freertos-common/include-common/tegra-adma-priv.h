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

#ifndef _TEGRA_ADMA_PRIV_H_
#define _TEGRA_ADMA_PRIV_H_

#include <stdbool.h>
#include <stdint.h>

#include <FreeRTOS.h>
#include <semphr.h>

#include <tegra-adma.h>

/*
 * There are 32 channels in HW, but the chance of non-audio SW needing that
 * many channels is slim at best. Restrict the number to reduce RAM usage of
 * the ID structure. We can always allow this to be overidden somehow if some
 * SW needs to use more channels, e.g. by defining this value here only if it
 * isn't already defined, or picking up the value from tegra-adma-hw.h.
 */
#define TEGRA_ADMA_NUM_CHANNELS 8

struct tegra_adma_id {
	const char *devname;
	uint32_t base_addr;
	uint32_t irq[TEGRA_ADMA_NUM_CHANNELS];
};

struct tegra_adma_channel {
	uint32_t irq;
	SemaphoreHandle_t irq_sem;
	bool synchronous;
	dma_callback *callback;
	void *callback_data;
};

struct tegra_adma_ctlr {
	/*
	 * The driver relies on this being the first field, since it casts
	 * pointers between structs tegra_adma_id and tegra_adma_ctlr.
	 */
	struct tegra_adma_id dma_id;
	struct tegra_adma_channel channels[TEGRA_ADMA_NUM_CHANNELS];
};

#endif
