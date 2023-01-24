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

#include <argpcdma.h>
#include <nvrm_drf.h>

#include <clk-tegra.h>
#include <delay.h>
#include <err-hook.h>
#include <irqs.h>
#include <macros.h>
#include <reg-access.h>
#include <tegra-gpcdma-priv.h>

static inline uint32_t dma_chan_readl(struct tegra_gpcdma_channel *dma_chan,
	uint32_t reg)
{
	reg += dma_chan->chan_base;

	return readl(reg);
}

static inline void dma_chan_writel(struct tegra_gpcdma_channel *dma_chan,
	uint32_t val, uint32_t reg)
{
	reg += dma_chan->chan_base;
	writel(val, reg);
}

static inline struct tegra_gpcdma_channel *tegra_gpcdma_chan(
		struct tegra_gpcdma_id *id, unsigned int chan_num)
{
	struct tegra_gpcdma_ctlr *dma_ctlr = (struct tegra_gpcdma_ctlr *)id;
	struct tegra_gpcdma_channel *chan = NULL;

	if (chan_num >= ARRAY_SIZE(dma_ctlr->channels))
		error_hook("invalid DMA channel");
	else
		chan = &dma_ctlr->channels[chan_num];

	return chan;
}

static void tegra_gpcdma_chan_irq(void *data);

int tegra_gpcdma_init(struct tegra_gpcdma_id *id)
{
	struct tegra_gpcdma_ctlr *dma_ctlr = (struct tegra_gpcdma_ctlr *)id;
	int i, ret = 0;

	for (i = 0; i < ARRAY_SSIZE(dma_ctlr->channels); i++) {
		struct tegra_gpcdma_channel *dma_chan = &dma_ctlr->channels[i];

		/* Set the channel register base address */
		dma_chan->chan_base = id->base_addr + i *
		(GPCDMA_CHANNEL_CH1_CSR_0 - GPCDMA_CHANNEL_CH0_CSR_0);
		dma_chan->irq = id->irqs[i];
		dma_chan->busy = false;

		dma_chan->irq_sem = xSemaphoreCreateBinary();
		if (dma_chan->irq_sem == NULL) {
			error_hook("XSemaphoreCreateBinary() failed");
			ret = -1;
			goto err_delete_semaphores;
		}

		irq_set_handler(id->irqs[i], tegra_gpcdma_chan_irq, dma_chan);
	}

#if !defined(_NV_BUILD_FPGA_) && !defined(_NV_BUILD_LINSIM_)
	/* Reset the GPCDMA controller */
	ret = tegra_clk_reset_pulse(dma_ctlr->id.rst, 2);
	if (ret) {
		error_hook("DMA controller reset failed");
		goto err_delete_semaphores;
	}
#endif

	return 0;
err_delete_semaphores:
	while (--i >= 0) {
		struct tegra_gpcdma_channel *dma_chan = &dma_ctlr->channels[i];

		irq_set_handler(dma_chan->irq, NULL, NULL);
		vSemaphoreDelete(dma_chan->irq_sem);
	}

	return ret;
}

int tegra_gpcdma_channel_init(struct tegra_gpcdma_id *id, int chan_num)
{
	(void) id; (void) chan_num;
	return 0;
}

int tegra_gpcdma_channel_deinit(struct tegra_gpcdma_id *id, int num)
{
	struct tegra_gpcdma_channel *dma_chan = tegra_gpcdma_chan(id, num);

	if (dma_chan == NULL)
		return -1;

	/* Return error if the channel is busy */
	if (dma_chan->busy) {
		error_hook("Can't deinit() a channel that's busy");
		return -1;
	}

	return 0;
}

static inline int tegra_gpcdma_get_dma_mode(struct tegra_gpcdma_xfer *transfer,
	uint32_t *dma_mode, bool *src_io, bool *dst_io)
{

	switch (transfer->direction) {
	case TEGRA_GPCDMA_XFER_DIR_MEM_TO_IO:
		*dma_mode = (transfer->en_flow_ctrl) ?
			NV_DRF_DEF(GPCDMA, CHANNEL_CH0_CSR, DMA_MODE, MEM2IO_FC) :
			NV_DRF_DEF(GPCDMA, CHANNEL_CH0_CSR, DMA_MODE, MEM2IO_NO_FC);
		*src_io = false;
		*dst_io = true;
		break;
	case TEGRA_GPCDMA_XFER_DIR_IO_TO_MEM:
		*dma_mode = (transfer->en_flow_ctrl) ?
			NV_DRF_DEF(GPCDMA, CHANNEL_CH0_CSR, DMA_MODE, IO2MEM_FC) :
			NV_DRF_DEF(GPCDMA, CHANNEL_CH0_CSR, DMA_MODE, IO2MEM_NO_FC);
		*src_io = true;
		*dst_io = false;
		break;
	case TEGRA_GPCDMA_XFER_DIR_MEM_TO_MEM:
		*dma_mode = NV_DRF_DEF(GPCDMA, CHANNEL_CH0_CSR, DMA_MODE, MEM2MEM);
		*src_io = false;
		*dst_io = false;
		break;
	default:
		error_hook("Invalid xfer direction");
		return -1;
	}

	return 0;
}

static inline int tegra_gpcdma_io_addr_map_wrap(int wrap, uint32_t *val)
{
	/* Not a power of 2; multiple bits set */
	if (wrap & (wrap - 1)) {
		error_hook("Invalid IO addr wrap");
		return -1;
	}

	*val = __builtin_ffs(wrap);
	if (*val > GPCDMA_CHANNEL_CH0_MMIO_SEQ_0_MMIO_ADDR_WRAP_DEFAULT_MASK) {
		error_hook("Invalid IO addr wrap");
		return -1;
	}

	return 0;
}

static inline int tegra_gpcdma_mc_addr_map_wrap(int wrap, uint32_t *val)
{
	/* Not a power of 2; multiple bits set */
	if (wrap & (wrap - 1)) {
		error_hook("Invalid mc addr wrap");
		return -1;
	}

	if (!wrap) {
		*val = 0;
		return 0;
	}

	*val = (__builtin_ffs(wrap) - 5);
	if (*val > GPCDMA_CHANNEL_CH0_MC_SEQ_0_MC_ADDR_WRAP0_DEFAULT_MASK) {
		error_hook("Invalid mc addr wrap");
		return -1;
	}

	return 0;
}

enum dma_status tegra_gpcdma_transfer(struct tegra_gpcdma_id *id,
	int num, struct tegra_gpcdma_xfer *transfer)
{
	struct tegra_gpcdma_channel *dma_chan = tegra_gpcdma_chan(id, num);
	int ret;
	uint32_t val, wcount, dma_mode, wrap_bits, burst_bits;
	uint32_t io_wrap, mc_src_wrap, mc_dst_wrap;
	bool wait, src_io, dst_io;

	if (dma_chan == NULL)
		return DMA_STATUS_NOT_INITIATED;

	/* Channel should be initialized and no on-going transfers */
	if (dma_chan->busy) {
		error_hook("Channel not ready for xfer");
		return DMA_STATUS_NOT_INITIATED;
	}

	dma_chan->synchronous = transfer->synchronous;
	if (dma_chan->synchronous) {
		/*
		 * Synchronous transfers wait on xfer completion before
		 * returning status. So, clear any pending IRQ notifications
		 * before starting a new synchronous transfer.
		 */
		xSemaphoreTake(dma_chan->irq_sem, 0);
		dma_chan->callback = NULL;
	} else {
		if (!transfer->callback) {
			error_hook("No callback for async xfer");
			return DMA_STATUS_NOT_INITIATED;
		}
		dma_chan->callback = transfer->callback;
		dma_chan->callback_param = transfer->callback_param;
	}

	ret = tegra_gpcdma_get_dma_mode(transfer, &dma_mode, &src_io, &dst_io);
	if (ret)
		return DMA_STATUS_NOT_INITIATED;

	/*
	 * Word count register takes argument in words. Value N means (N + 1)
	 * words xfer request size. For IO2MEM or MEM2IO transfers, transfer
	 * size should be a multiple of MMIO burst length.
	 */
	burst_bits = (transfer->burst_size << 2) - 1;
	if ((transfer->xfer_count & burst_bits) && (src_io || dst_io)) {
		error_hook("Invalid transfer length");
		return DMA_STATUS_NOT_INITIATED;
	}
	wcount = transfer->xfer_count >> 2;
	dma_chan_writel(dma_chan, (wcount - 1), GPCDMA_CHANNEL_CH0_BCOUNT_0);

	/* Set MC request count, MC burst size and MC address wrap settings */
	// FIXME: Should the burst size here be driven by transfer->burst_size
	// and feed into burst_bits checks above?
	val = NV_DRF_DEF(GPCDMA, CHANNEL_CH0_MC_SEQ, MC_BURST, DMA_BURST_16WORDS) |
		NV_DRF_DEF(GPCDMA, CHANNEL_CH0_MC_SEQ, MC_REQ_CNT, DEFAULT);
	if (!src_io) {
		ret = tegra_gpcdma_mc_addr_map_wrap(transfer->src_addr_wrap,
			&mc_src_wrap);
		if (ret)
			return DMA_STATUS_NOT_INITIATED;
	} else
		mc_src_wrap = 0;
	if (!dst_io) {
		ret = tegra_gpcdma_mc_addr_map_wrap(transfer->dst_addr_wrap,
			&mc_dst_wrap);
		if (ret)
			return DMA_STATUS_NOT_INITIATED;
	} else
		mc_dst_wrap = 0;
	val |= NV_DRF_NUM(GPCDMA, CHANNEL_CH0_MC_SEQ, MC_ADDR_WRAP0, mc_src_wrap) |
		NV_DRF_NUM(GPCDMA, CHANNEL_CH0_MC_SEQ, MC_ADDR_WRAP1, mc_dst_wrap);
	dma_chan_writel(dma_chan, val, GPCDMA_CHANNEL_CH0_MC_SEQ_0);

	/*
	 * MMIO sequence needs to be programmed only for MEM2IO or IO2MEM
	 * transfers. Set IO buswidth, IO burst size and address wrap settings.
	 */
	if (src_io || dst_io) {
		val = NV_DRF_NUM(GPCDMA, CHANNEL_CH0_MMIO_SEQ, MMIO_BUS_WIDTH,
			transfer->bus_width);
		val |= NV_DRF_NUM(GPCDMA, CHANNEL_CH0_MMIO_SEQ, MMIO_BURST,
			(transfer->burst_size - 1));
		if (src_io) {
			ret = tegra_gpcdma_io_addr_map_wrap(
				transfer->src_addr_wrap, &io_wrap);
			if (ret)
				return DMA_STATUS_NOT_INITIATED;
			wrap_bits = transfer->src_addr_wrap ?
				transfer->src_addr_wrap - 1 : 0;
		} else {
			ret = tegra_gpcdma_io_addr_map_wrap(
				transfer->dst_addr_wrap, &io_wrap);
			if (ret)
				return DMA_STATUS_NOT_INITIATED;
			wrap_bits = transfer->dst_addr_wrap ?
				transfer->dst_addr_wrap - 1 : 0;
		}
		val |= NV_DRF_NUM(GPCDMA, CHANNEL_CH0_MMIO_SEQ, MMIO_ADDR_WRAP, io_wrap);
		dma_chan_writel(dma_chan, val, GPCDMA_CHANNEL_CH0_MMIO_SEQ_0);
	} else {
		dma_chan_writel(dma_chan, 0, GPCDMA_CHANNEL_CH0_MMIO_SEQ_0);
		wrap_bits = 0;
	}

	/*
	 * IO address needs to be aligned with IO address wrap size.
	 * MC address needs to be word aligned.
	 */
	if (((uint32_t)transfer->src_addr & 0x3) || (src_io &&
			((uint32_t)transfer->src_addr & wrap_bits))) {
		error_hook("Unaligned src address");
		return DMA_STATUS_NOT_INITIATED;
	}
	if (((uint32_t)transfer->dst_addr & 0x3) || (dst_io &&
			((uint32_t)transfer->dst_addr & wrap_bits))) {
		error_hook("Unaligned dst address");
		return DMA_STATUS_NOT_INITIATED;
	}
	dma_chan_writel(dma_chan, (uint32_t)transfer->src_addr,
		GPCDMA_CHANNEL_CH0_SRC_PTR_0);
	dma_chan_writel(dma_chan, (uint32_t)transfer->dst_addr,
		GPCDMA_CHANNEL_CH0_DST_PTR_0);

	/*
	 * Set DMA mode, channel IRQ mask, channel interrupt, once/continuous
	 * mode, channel priority and enable the channel. If flow control is
	 * enabled, set the slave request ID.
	 */
	val = dma_mode;
	if (transfer->en_flow_ctrl) {
		if (transfer->slave_req > NV_FIELD_MASK(
				GPCDMA_CHANNEL_CH0_CSR_0_REQ_SEL_RANGE)) {
			error_hook("Invalid request select");
			return DMA_STATUS_NOT_INITIATED;
		}
		val |= NV_DRF_NUM(GPCDMA, CHANNEL_CH0_CSR, REQ_SEL,
			transfer->slave_req);
	}
	val |= NV_DRF_DEF(GPCDMA, CHANNEL_CH0_CSR, IRQ_MASK, ENABLE) |
		NV_DRF_DEF(GPCDMA, CHANNEL_CH0_CSR, IE_EOC, ENABLE) |
		NV_DRF_DEF(GPCDMA, CHANNEL_CH0_CSR, ONCE, SINGLE_BLOCK) |
		NV_DRF_DEF(GPCDMA, CHANNEL_CH0_CSR, WEIGHT, DEFAULT) |
		NV_DRF_DEF(GPCDMA, CHANNEL_CH0_CSR, ENB, ENABLE);
	dma_chan->busy = true;
	dma_chan_writel(dma_chan, val, GPCDMA_CHANNEL_CH0_CSR_0);

	wait = dma_chan->synchronous;
	irq_enable(dma_chan->irq);
	if (!wait)
		return DMA_STATUS_EXECUTING;

	if (xSemaphoreTake(dma_chan->irq_sem,
			transfer->timeout / portTICK_PERIOD_MS) != pdTRUE) {
		error_hook("timeout");
		tegra_gpcdma_abort(id, num);
		return DMA_STATUS_TIMEOUT;
	}

	return DMA_STATUS_COMPLETE;
}

int tegra_gpcdma_get_count_xferred(struct tegra_gpcdma_id *id, int num)
{
	struct tegra_gpcdma_channel *dma_chan = tegra_gpcdma_chan(id, num);
	uint32_t csre, xfer_count;

	if (dma_chan == NULL)
		return -1;

	csre = dma_chan_readl(dma_chan, GPCDMA_CHANNEL_CH0_CSRE_0);
	csre |= NV_DRF_DEF(GPCDMA, CHANNEL_CH0_CSRE, DMA_ACTIVITY, PAUSE);
	dma_chan_writel(dma_chan, csre, GPCDMA_CHANNEL_CH0_CSRE_0);

	xfer_count = dma_chan_readl(dma_chan,
		GPCDMA_CHANNEL_CH0_DMA_BYTE_STA_0);

	csre &= NV_DRF_DEF(GPCDMA, CHANNEL_CH0_CSRE, DMA_ACTIVITY, RESUME);
	dma_chan_writel(dma_chan, csre, GPCDMA_CHANNEL_CH0_CSRE_0);

	return xfer_count * 4;
}

int tegra_gpcdma_abort(struct tegra_gpcdma_id *id, int num)
{
	struct tegra_gpcdma_channel *dma_chan = tegra_gpcdma_chan(id, num);

	if (dma_chan == NULL)
		return -1;

	/* Disable IRQ and then disable the channel */
	irq_disable(dma_chan->irq);
	dma_chan_writel(dma_chan, 0, GPCDMA_CHANNEL_CH0_CSR_0);
	dma_chan->busy = false;
	/*
	 * This function can be called internally from tegra_gpcdma_transfer()
	 * for synchronous xfers, and there's no callback for those.
	 */
	if (dma_chan->callback)
		dma_chan->callback(dma_chan->callback_param,
			DMA_STATUS_ABORTED);

	return 0;
}

static void tegra_gpcdma_chan_irq(void *data)
{
	struct tegra_gpcdma_channel *dma_chan = data;
	uint32_t status;
	BaseType_t higher_prio_task_woken = pdFALSE;

	irq_disable(dma_chan->irq);
	dma_chan->busy = false;

	/* Clear the DMA interrupt first */
	status = dma_chan_readl(dma_chan, GPCDMA_CHANNEL_CH0_STA_0);
	if (NV_DRF_VAL(GPCDMA, CHANNEL_CH0_STA, ISE_EOC, status))
		dma_chan_writel(dma_chan, status, GPCDMA_CHANNEL_CH0_STA_0);

	if (dma_chan->synchronous) {
		xSemaphoreGiveFromISR(dma_chan->irq_sem,
					&higher_prio_task_woken);
		portYIELD_FROM_ISR(higher_prio_task_woken);
	} else {
		dma_chan->callback(dma_chan->callback_param,
			DMA_STATUS_COMPLETE);
	}
}

void tegra_gpcdma_irq(struct tegra_gpcdma_id *id, int chan_num)
{
	struct tegra_gpcdma_ctlr *dma_ctlr = (struct tegra_gpcdma_ctlr *)id;

	tegra_gpcdma_chan_irq(&dma_ctlr->channels[chan_num]);
}
