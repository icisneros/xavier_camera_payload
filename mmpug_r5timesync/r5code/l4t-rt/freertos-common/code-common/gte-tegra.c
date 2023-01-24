/*
 * Copyright (c) 2015-2016, NVIDIA CORPORATION.  All rights reserved.
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

#include <stdio.h>
#include <stdint.h>

#include <argte_aps.h>
#include <nvrm_drf.h>

#include <delay.h>
#include <err-hook.h>
#include <gte-tegra-priv.h>
#include <irqs.h>
#include <macros.h>
#include <reg-access.h>

#define GTE_APS_SLICE_SIZE	(BASE_ADDRESS_GTE_APS_SLICE1 - BASE_ADDRESS_GTE_APS_SLICE0)

struct tegra_gte_fifo_msg tegra_gte_msg;

static inline uint32_t tegra_gte_readl(struct tegra_gte_id *id, uint32_t reg)
{
	return readl(id->base_addr + reg);
}

static inline void tegra_gte_writel(struct tegra_gte_id *id, uint32_t val, uint32_t reg)
{
	writel(val, id->base_addr + reg);
}

void tegra_gte_slice_set_enable_mask(struct tegra_gte_id *id, uint8_t slice, uint32_t bitmap)
{
	tegra_gte_writel(id, bitmap, (slice * GTE_APS_SLICE_SIZE) + GTE_APS_SLICE0_TETEN_0);
}

static void tegra_gte_get_status(struct tegra_gte_id *id, uint8_t *occupancy, bool *int_sts)
{
	uint32_t val;

	val = tegra_gte_readl(id, GTE_APS_TESTATUS_0);

	*int_sts = NV_DRF_VAL(GTE_APS, TESTATUS, INTERRUPT, val);
	*occupancy = NV_DRF_VAL(GTE_APS, TESTATUS, OCCUPANCY, val);
}

static void tegra_gte_read_fifo(struct tegra_gte_id *id, struct tegra_gte_ts *ts)
{
	uint32_t encv, tsc_hi, tsc_lo;

	encv = tegra_gte_readl(id, GTE_APS_TEENCV_0);

	tsc_hi = tegra_gte_readl(id, GTE_APS_TETSCH_0);
	tsc_lo = tegra_gte_readl(id, GTE_APS_TETSCL_0);
	ts->tsc = (((uint64_t)tsc_hi) << 32) | tsc_lo;

	tegra_gte_writel(id, GTE_APS_TECMD_0_CMD_POP, GTE_APS_TECMD_0);

	ts->slice = NV_DRF_VAL(GTE_APS, TEENCV, SLICE, encv);
	ts->bit_index = NV_DRF_VAL(GTE_APS, TEENCV, BITINDEX, encv);
	ts->bit_dir = NV_DRF_VAL(GTE_APS, TEENCV, FALLING, encv);
	ts->invalid = NV_DRF_VAL(GTE_APS, TEENCV, INVALID, encv);
}

void tegra_gte_irq(void *data)
{
	struct tegra_gte_id *id = data;
	/* struct tegra_gte_id assumed to be first field in tegra_gte */
	struct tegra_gte *gte_dev = (struct tegra_gte *)id;
	struct tegra_gte_ts ts_data;
	uint8_t occupancy, i;
	bool int_sts;

	tegra_gte_get_status(id, &occupancy, &int_sts);
	for (i = 0; i < occupancy; i++) {
		tegra_gte_read_fifo(id, &ts_data);
		if (gte_dev->irq_callback)
			gte_dev->irq_callback(gte_dev->irq_data, &ts_data);
	}
}

void tegra_gte_dma_complete_handler(void *callback_param,
				    enum dma_status status)
{
	/* struct tegra_gte_id assumed to be first field in tegra_gte */
	struct tegra_gte *gte_dev = (struct tegra_gte *)callback_param;
	struct tegra_gte_ts ts;
	uint32_t tsc_lo, tsc_hi, acv;

	if (status != DMA_STATUS_COMPLETE) {
		error_hook("dma transter failed");
		return;
	}

	ts.invalid = 0;
	tsc_lo = tegra_gte_msg.tsc_lo;
	tsc_hi = tegra_gte_msg.tsc_hi;
	ts.tsc = (((uint64_t)tsc_hi) << 32) | tsc_lo;
	ts.slice = NV_DRF_VAL(GTE_APS, TESRC, SLICE, tegra_gte_msg.src);
	acv = tegra_gte_msg.pcv ^ tegra_gte_msg.ccv;

	while (acv) {
		ts.bit_index = __builtin_ctz(acv);
		if ((tegra_gte_msg.pcv >> ts.bit_index) & BIT(0))
			ts.bit_dir = 1;
		else
			ts.bit_dir = 0;
		gte_dev->irq_callback(gte_dev->irq_data, &ts);
		acv &= ~BIT(ts.bit_index);
	}

	gte_dev->dma_xfer((void *)gte_dev->id.base_addr, (void *)&tegra_gte_msg,
			  sizeof(tegra_gte_msg), 8, 8);
}

int tegra_gte_setup(struct tegra_gte_id *id, uint32_t occupancy_threshold,
		    tegra_gte_callback irq_callback,
		    void *irq_data,
		    tegra_gte_dma_xfer dma_xfer,
		    bool dma_en)
{
	/* struct tegra_gte_id assumed to be first field in tegra_gte */
	struct tegra_gte *gte_dev = (struct tegra_gte *)id;
	uint32_t val;

	if (occupancy_threshold &
	    ~GTE_APS_TECTRL_0_OCCUPANCYTHRESHOLD_DEFAULT_MASK) {
		error_hook("occupancy_threshold exceeds MAX value");
		return 1;
	}

	if (!irq_callback) {
		error_hook("irq_callback is NULL");
		return 1;
	}

	gte_dev->irq_callback = irq_callback;
	gte_dev->irq_data = irq_data;

	if (dma_en) {
		if (!dma_xfer) {
			error_hook("dma_xfer is NULL");
			return 1;
		}
		gte_dev->dma_xfer = dma_xfer;
		irq_disable(id->irq);

		val =
			NV_DRF_NUM(GTE_APS, TECTRL, ENABLE, 1) |
			NV_DRF_NUM(GTE_APS, TECTRL, INTERRUPTENABLE, 1) |
			NV_DRF_NUM(GTE_APS, TECTRL, AUTOADVENABLE, 1) |
			NV_DRF_NUM(GTE_APS, TECTRL, AUTOADVADDR, 3) |
			NV_DRF_NUM(GTE_APS, TECTRL, OCCUPANCYTHRESHOLD, 1);
		tegra_gte_writel(id, val, GTE_APS_TECTRL_0);

		gte_dev->dma_xfer((void *)id->base_addr, (void *)&tegra_gte_msg,
			  sizeof(tegra_gte_msg), 8, 8);
	}
	else {
		val =
			NV_DRF_NUM(GTE_APS, TECTRL, ENABLE, 1) |
			NV_DRF_NUM(GTE_APS, TECTRL, INTERRUPTENABLE, 1) |
			NV_DRF_NUM(GTE_APS, TECTRL, AUTOADVENABLE, 0) |
			NV_DRF_NUM(GTE_APS, TECTRL, AUTOADVADDR, 0) |
			NV_DRF_NUM(GTE_APS, TECTRL, OCCUPANCYTHRESHOLD, occupancy_threshold);
		tegra_gte_writel(id, val, GTE_APS_TECTRL_0);

		irq_set_handler(id->irq, tegra_gte_irq, id);
		irq_enable(id->irq);
	}

	return 0;
}
