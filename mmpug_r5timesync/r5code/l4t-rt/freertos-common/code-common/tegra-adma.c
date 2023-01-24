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

#include <aradma.h>
#include <nvrm_drf.h>

#include <delay.h>
#include <err-hook.h>
#include <irqs.h>
#include <macros.h>
#include <reg-access.h>
#include <tegra-adma-priv.h>

static inline uint32_t tegra_adma_readl(struct tegra_adma_id *id, uint32_t reg)
{
	reg += id->base_addr;
	return readl(reg);
}

static inline void tegra_adma_writel(struct tegra_adma_id *id, uint32_t val,
	uint32_t reg)
{
	reg += id->base_addr;
	writel(val, reg);
}

static inline uint32_t tegra_adma_ch_reg(int chan, uint32_t reg)
{
	return reg + (chan * (ADMA_PAGE1_CH2_CMD_0 - ADMA_PAGE1_CH1_CMD_0));
}

static inline int tegra_adma_spin_for_reg(struct tegra_adma_id *id,
	uint32_t mask, uint32_t reg, bool wait_for_set)
{
	uint64_t ts = get_time_ticks();

	for (;;) {
		uint32_t val = tegra_adma_readl(id, reg);
		if (!wait_for_set)
			val = ~val;
		if (val & mask)
			return 0;
		if (get_time_delta_us(ts) > 100) {
			error_hook("timeout");
			return -1;
		}
	}
}

static inline struct tegra_adma_channel *tegra_adma_chan(
		struct tegra_adma_id *id, unsigned int chan_num)
{
	struct tegra_adma_ctlr *ctlr = (struct tegra_adma_ctlr *)id;
	struct tegra_adma_channel *ch = NULL;

	if (chan_num >= ARRAY_SIZE(ctlr->channels))
		error_hook("invalid DMA channel");
	else
		ch = &ctlr->channels[chan_num];

	return ch;
}

static void tegra_adma_chan_irq(void *data);

int tegra_adma_init(struct tegra_adma_id *id)
{
	struct tegra_adma_ctlr *ctlr = (struct tegra_adma_ctlr *)id;
	size_t i;
	uint32_t val;
	int ret;

	for (i = 0; i < ARRAY_SIZE(ctlr->channels); i++) {
		struct tegra_adma_channel *ch = &ctlr->channels[i];

		ch->irq = id->irq[i];
		ch->irq_sem = xSemaphoreCreateBinary();
		if (ch->irq_sem == NULL) {
			error_hook("xSemaphoreCreateBinary() failed");
			while (ch > ctlr->channels) {
				ch--;
				irq_set_handler(ch->irq, NULL, NULL);
				vSemaphoreDelete(ch->irq_sem);
			}
			return -1;
		}

		irq_set_handler(ch->irq, tegra_adma_chan_irq, ch);
	}

	val = NV_DRF_DEF(ADMA, GLOBAL_SOFT_RESET, ENABLE, TRUE);
	tegra_adma_writel(id, val, ADMA_GLOBAL_SOFT_RESET_0);
	ret = tegra_adma_spin_for_reg(id, val, ADMA_GLOBAL_SOFT_RESET_0, false);
	if (ret)
		return ret;

	val = NV_DRF_DEF(ADMA, GLOBAL_CMD, GLOBAL_ENABLE, TRUE);
	tegra_adma_writel(id, val, ADMA_GLOBAL_CMD_0);

	return 0;
}

int tegra_adma_channel_init(struct tegra_adma_id *id, int chan)
{
	struct tegra_adma_ctlr *ctlr = (struct tegra_adma_ctlr *)id;
	uint32_t reg = tegra_adma_ch_reg(chan, ADMA_PAGE1_CH1_SOFT_RESET_0);
	uint32_t val = NV_DRF_DEF(ADMA, PAGE1_CH1_SOFT_RESET, ENABLE, TRUE);

	if ((unsigned int)chan >= ARRAY_SIZE(ctlr->channels)) {
		error_hook("invalid DMA channel");
		return -1;
	}

	tegra_adma_writel(id, val, reg);
	return tegra_adma_spin_for_reg(id, val, reg, false);
}

static inline int tegra_adma_map_dir(enum tegra_adma_transfer_direction dir,
	uint32_t *val)
{
	switch (dir) {
	case TEGRA_ADMA_DIR_MEM_TO_MEM:
		*val = ADMA_PAGE1_CH1_CTRL_0_TRANSFER_DIRECTION_MEMORY_TO_MEMORY;
		return 0;
	case TEGRA_ADMA_DIR_AHUB_TO_MEM:
		*val = ADMA_PAGE1_CH1_CTRL_0_TRANSFER_DIRECTION_AHUB_TO_MEMORY;
		return 0;
	case TEGRA_ADMA_DIR_MEM_TO_AHUB:
		*val = ADMA_PAGE1_CH1_CTRL_0_TRANSFER_DIRECTION_MEMORY_TO_AHUB;
		return 0;
	case TEGRA_ADMA_DIR_AHUB_TO_AHUB:
		*val = ADMA_PAGE1_CH1_CTRL_0_TRANSFER_DIRECTION_AHUB_TO_AHUB;
		return 0;
	default:
		error_hook("invalid transfer direction");
		return -1;
	}
}

static inline int tegra_adma_map_wrap(int wrap, uint32_t *val)
{
	/* Not a power of 2; multiple bits set */
	if (wrap & (wrap - 1)) {
		error_hook("invalid wrap");
		return -1;
	}

	*val = __builtin_ffs(wrap);
	if (*val > NV_FIELD_MASK(ADMA_PAGE1_CH1_CONFIG_0_SOURCE_ADDR_WRAP_RANGE)) {
		error_hook("invalid wrap");
		return -1;
	}

	return 0;
}

enum dma_status tegra_adma_transfer(struct tegra_adma_id *id, int chan,
	struct tegra_adma_transfer *transfer)
{
	struct tegra_adma_channel *ch = tegra_adma_chan(id, chan);
	int ret;
	uint32_t dir, burst_bits, src_wrap, targ_wrap;
	uint32_t val;
	bool wait;

	if (ch == NULL)
		return -1;

	ch->synchronous = transfer->synchronous;
	if (ch->synchronous) {
		xSemaphoreTake(ch->irq_sem, 0);
	} else {
		if (!transfer->callback) {
			error_hook("NULL callback");
			return DMA_STATUS_NOT_INITIATED;
		}
		ch->callback = transfer->callback;
		ch->callback_data = transfer->callback_data;
	}

	val = NV_DRF_DEF(ADMA, PAGE1_CH1_INT_CLEAR, TRANSFER_DONE, TRUE);
	tegra_adma_writel(id, val,
		tegra_adma_ch_reg(chan, ADMA_PAGE1_CH1_INT_CLEAR_0));

	ret = tegra_adma_map_dir(transfer->direction, &dir);
	if (ret)
		return DMA_STATUS_NOT_INITIATED;
	val =
		NV_DRF_NUM(ADMA, PAGE1_CH1_CTRL, TRANSFER_DIRECTION, dir) |
		NV_DRF_DEF(ADMA, PAGE1_CH1_CTRL, TRANSFER_MODE, ONCE) |
		NV_DRF_DEF(ADMA, PAGE1_CH1_CTRL, FLOWCTRL_TYPE, THRESHOLD_BASED);
	if (transfer->enable_flow_ctrl) {
		val |= NV_DRF_DEF(ADMA, PAGE1_CH1_CTRL, FLOWCTRL_ENABLE, TRUE);
		if (transfer->rx_slave_request >
			NV_FIELD_MASK(ADMA_PAGE1_CH1_CTRL_0_RX_REQUEST_SELECT_RANGE)) {
			error_hook("invalid RX req sel");
			return DMA_STATUS_NOT_INITIATED;
		}
		val |= NV_DRF_NUM(ADMA, PAGE1_CH1_CTRL, RX_REQUEST_SELECT,
			transfer->rx_slave_request);
		if (transfer->tx_slave_request >
			NV_FIELD_MASK(ADMA_PAGE1_CH1_CTRL_0_TX_REQUEST_SELECT_RANGE)) {
			error_hook("invalid TX req sel");
			return DMA_STATUS_NOT_INITIATED;
		}
		val |= NV_DRF_NUM(ADMA, PAGE1_CH1_CTRL, TX_REQUEST_SELECT,
			transfer->tx_slave_request);
	}
	tegra_adma_writel(id, val,
		tegra_adma_ch_reg(chan, ADMA_PAGE1_CH1_CTRL_0));

	if (!transfer->burst_size) {
		error_hook("invalid burst_size");
		return DMA_STATUS_NOT_INITIATED;
	}
	if ((transfer->burst_size - 1) >
		NV_FIELD_MASK(ADMA_PAGE1_CH1_CONFIG_0_BURST_SIZE_RANGE)) {
		error_hook("invalid burst_size");
		return DMA_STATUS_NOT_INITIATED;
	}
	burst_bits = (transfer->burst_size * 4) - 1;
	if (transfer->source_addr_wrap & burst_bits) {
		error_hook("invalid source_addr_wrap");
		return DMA_STATUS_NOT_INITIATED;
	}
	ret = tegra_adma_map_wrap(transfer->source_addr_wrap, &src_wrap);
	if (ret)
		return DMA_STATUS_NOT_INITIATED;
	if (transfer->target_addr_wrap & burst_bits) {
		error_hook("invalid target_addr_wrap");
		return DMA_STATUS_NOT_INITIATED;
	}
	ret = tegra_adma_map_wrap(transfer->target_addr_wrap, &targ_wrap);
	if (ret)
		return DMA_STATUS_NOT_INITIATED;
	val =
		NV_DRF_NUM(ADMA, PAGE1_CH1_CONFIG, BURST_SIZE, transfer->burst_size - 1) |
		NV_DRF_NUM(ADMA, PAGE1_CH1_CONFIG, SOURCE_ADDR_WRAP, src_wrap) |
		NV_DRF_NUM(ADMA, PAGE1_CH1_CONFIG, TARGET_ADDR_WRAP, targ_wrap) |
		NV_DRF_NUM(ADMA, PAGE1_CH1_CONFIG, OUTSTANDING_REQUESTS, 1) |
		NV_DRF_NUM(ADMA, PAGE1_CH1_CONFIG, WEIGHT_FOR_WRR, 1);
	tegra_adma_writel(id, val,
		tegra_adma_ch_reg(chan, ADMA_PAGE1_CH1_CONFIG_0));

	if ((uint32_t)transfer->source & burst_bits) {
		error_hook("invalid source addr");
		return DMA_STATUS_NOT_INITIATED;
	}
	tegra_adma_writel(id, (uint32_t)transfer->source,
		tegra_adma_ch_reg(chan, ADMA_PAGE1_CH1_LOWER_SOURCE_ADDR_0));

	if ((uint32_t)transfer->target & burst_bits) {
		error_hook("invalid target addr");
		return DMA_STATUS_NOT_INITIATED;
	}
	tegra_adma_writel(id, (uint32_t)transfer->target,
		tegra_adma_ch_reg(chan, ADMA_PAGE1_CH1_LOWER_TARGET_ADDR_0));

	if ((uint32_t)transfer->transfer_count & burst_bits) {
		error_hook("invalid transfer_count");
		return DMA_STATUS_NOT_INITIATED;
	}
	if ((uint32_t)transfer->transfer_count >
		NV_FIELD_MASK(ADMA_PAGE1_CH1_TC_0_TRANSFER_COUNT_RANGE)) {
		error_hook("invalid transfer_count");
		return DMA_STATUS_NOT_INITIATED;
	}
	tegra_adma_writel(id, transfer->transfer_count,
		tegra_adma_ch_reg(chan, ADMA_PAGE1_CH1_TC_0));

	tegra_adma_writel(id, 0,
		tegra_adma_ch_reg(chan, ADMA_PAGE1_CH1_LOWER_DESC_ADDR_0));

	tegra_adma_writel(id,
		NV_DRF_DEF(ADMA, PAGE1_CH1_CMD, TRANSFER_ENABLE, TRUE),
		tegra_adma_ch_reg(chan, ADMA_PAGE1_CH1_CMD_0));

	wait = ch->synchronous;
	irq_enable(ch->irq);
	if (!wait)
		return DMA_STATUS_EXECUTING;

	if (xSemaphoreTake(ch->irq_sem,
			transfer->timeout / portTICK_PERIOD_MS) != pdTRUE) {
		error_hook("timeout");
		tegra_adma_abort(id, chan);
		return DMA_STATUS_TIMEOUT;
	}

	return DMA_STATUS_COMPLETE;
}

int tegra_adma_get_count_xferd(struct tegra_adma_id *id, int chan)
{
	struct tegra_adma_ctlr *ctlr = (struct tegra_adma_ctlr *)id;
	uint32_t ctrl, tc, tc_status;
	int ret;

	if ((unsigned int)chan >= ARRAY_SIZE(ctlr->channels)) {
		error_hook("invalid DMA channel");
		return -1;
	}

	ctrl = tegra_adma_readl(id,
		tegra_adma_ch_reg(chan, ADMA_PAGE1_CH1_CTRL_0));
	ctrl |= NV_DRF_DEF(ADMA, PAGE1_CH1_CTRL, TRANSFER_PAUSE, TRUE);
	tegra_adma_writel(id, ctrl,
		tegra_adma_ch_reg(chan, ADMA_PAGE1_CH1_CTRL_0));

	ret = tegra_adma_spin_for_reg(id,
		NV_DRF_DEF(ADMA, PAGE1_CH1_STATUS, TRANSFER_PAUSED, TRUE),
		tegra_adma_ch_reg(chan, ADMA_PAGE1_CH1_STATUS_0), true);

	tc_status = tegra_adma_readl(id,
		tegra_adma_ch_reg(chan, ADMA_PAGE1_CH1_TC_STATUS_0));

	tc = tegra_adma_readl(id,
		tegra_adma_ch_reg(chan, ADMA_PAGE1_CH1_TC_0));

	ctrl &= ~NV_DRF_DEF(ADMA, PAGE1_CH1_CTRL, TRANSFER_PAUSE, TRUE);
	tegra_adma_writel(id, ctrl,
		tegra_adma_ch_reg(chan, ADMA_PAGE1_CH1_CTRL_0));

	if (ret)
		return -1;

	return tc - tc_status;
}

int tegra_adma_abort(struct tegra_adma_id *id, int chan)
{
	struct tegra_adma_channel *ch = tegra_adma_chan(id, chan);
	uint32_t ctrl;

	if (ch == NULL)
		return -1;

	ctrl = tegra_adma_readl(id,
		tegra_adma_ch_reg(chan, ADMA_PAGE1_CH1_CTRL_0));
	ctrl |= NV_DRF_DEF(ADMA, PAGE1_CH1_CTRL, TRANSFER_PAUSE, TRUE);
	tegra_adma_writel(id, ctrl,
		tegra_adma_ch_reg(chan, ADMA_PAGE1_CH1_CTRL_0));

	tegra_adma_spin_for_reg(id,
		NV_DRF_DEF(ADMA, PAGE1_CH1_STATUS, TRANSFER_PAUSED, TRUE),
		tegra_adma_ch_reg(chan, ADMA_PAGE1_CH1_STATUS_0), true);

	tegra_adma_channel_init(id, chan);

	ch->callback(ch->callback_data, DMA_STATUS_ABORTED);

	return 0;
}

static void tegra_adma_chan_irq(void *data)
{
	struct tegra_adma_channel *ch = data;
	BaseType_t higher_prio_task_woken = pdFALSE;

	irq_disable(ch->irq);

	if (ch->synchronous) {
		xSemaphoreGiveFromISR(ch->irq_sem, &higher_prio_task_woken);
		portYIELD_FROM_ISR(higher_prio_task_woken);
	} else {
		ch->callback(ch->callback_data, DMA_STATUS_COMPLETE);
	}
}
