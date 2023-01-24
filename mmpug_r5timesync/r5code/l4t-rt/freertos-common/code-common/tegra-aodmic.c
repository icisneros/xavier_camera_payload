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

#include <stdlib.h>

#include <argpcdma_ao.h>
#include <araodmic.h>
#include <nvrm_drf.h>
#include <dma.h>

#include <clk-tegra.h>
#include <delay.h>
#include <err-hook.h>
#include <irqs.h>
#include <macros.h>
#include <reg-access.h>
#include <tegra-aodmic-priv.h>

static inline uint32_t aodmic_readl(struct tegra_aodmic_ctlr *ctlr,
                                    uint32_t offset)
{
	return readl(ctlr->id.base_addr + offset);
}

static inline void aodmic_writel(struct tegra_aodmic_ctlr *ctlr,
                                 uint32_t value, uint32_t offset)
{
	writel(value, ctlr->id.base_addr + offset);
}

/* Callback from GPCDMA upon completion of transfer */
static void tegra_aodmic_gpcdma_callback(void *callback_param,
                                         enum dma_status status)
{
	struct tegra_aodmic_ctlr *ctlr;
	BaseType_t free_buf_avail;
	BaseType_t higher_prio_task_woken = pdFALSE;
	enum dma_status dma_ret;
	void *filled_buf;

	if (status != DMA_STATUS_COMPLETE) {
		if (status != DMA_STATUS_ABORTED)
			error_hook("DMA transfer failed");

		return;
	}

	ctlr = (struct tegra_aodmic_ctlr *)callback_param;

	/* Address of filled buffer */
	filled_buf = ctlr->dma_xfer.dst_addr;

	/* Acquire next free buffer so that   */
	/* next DMA transfer can be triggered */
	free_buf_avail = xSemaphoreTakeFromISR(ctlr->xSemaphore_free_buf,
	                                       &higher_prio_task_woken);

	/* No free buffer indicates overflow due to low/no */
	/* consumption, halt DMA transfers in such case    */
	if (!free_buf_avail) {
		ctlr->dma_running = 0;
		error_hook("DMA buffer overflow");
	}

	/* Update write position in data buffer */
	/* and trigger next DMA transfer        */
	if (ctlr->dma_running) {
		ctlr->free_pos = (ctlr->free_pos + 1) % ctlr->num_periods;
		ctlr->dma_xfer.dst_addr =
		        ctlr->data + (ctlr->free_pos * ctlr->period_bytes);
		dma_ret = tegra_gpcdma_transfer(ctlr->gpcdma_id,
		                                ctlr->dma_chan_num,
		                                &ctlr->dma_xfer);
		if (dma_ret != DMA_STATUS_EXECUTING)
			error_hook("Error in tegra_gpcdma_transfer");
	}

	/* Post available buffer for consumption */
	xQueueSendToBackFromISR(ctlr->xQueue_filled_buf, &filled_buf,
	                        &higher_prio_task_woken);

	portYIELD_FROM_ISR(higher_prio_task_woken);
}

int tegra_aodmic_open(struct tegra_aodmic_id *id,
                      struct tegra_aodmic_config *config)
{
	struct tegra_aodmic_ctlr *ctlr = (struct tegra_aodmic_ctlr *)id;
	uint32_t sample_rate, osr, num_channels, samp_bytes, period_bytes;
	uint32_t ctrl_reg, fifo_ctrl_reg, dbg_ctrl_reg, enable_reg;
	int32_t status, ret_val = 0;

	/* Initialize memory pointers in ctlr to NULL */
	ctlr->data                = NULL;
	ctlr->xSemaphore_free_buf = NULL;
	ctlr->xQueue_filled_buf   = NULL;

	if (config == NULL) {
		error_hook("Configuration not available");
		ret_val = TEGRA_AODMIC_ERR_CONFIGURATION_NA;
		goto err_exit;
	}

	/* Setup clock as per sample rate configuration */
	/* Clock = OSR * sample_rate                    */
	switch (config->sample_rate) {
	case TEGRA_AODMIC_RATE_8KHZ:    sample_rate =  8000; break;
	case TEGRA_AODMIC_RATE_16KHZ:   sample_rate = 16000; break;
	case TEGRA_AODMIC_RATE_44KHZ:   sample_rate = 44100; break;
	case TEGRA_AODMIC_RATE_48KHZ:   sample_rate = 48000; break;

	default:
		error_hook("Invalid sample rate configuration");
		ret_val = TEGRA_AODMIC_ERR_INVALID_SAMPLE_RATE;
		goto err_exit;
	}
	ctlr->sample_rate = sample_rate;
	/* OSR is tied to OSR64 for AODMIC */
	osr = 1 << (AODMIC_CTRL_0_OSR_OSR64 + 6);
	status = tegra_clk_set_rate(ctlr->id.clk, (osr * sample_rate));
	if (status) {
		error_hook("Unable to set CLK (check sample rate)");
		ret_val = TEGRA_AODMIC_ERR_CLK_RATE_ERROR;
		goto err_exit;
	}

#ifndef _NV_BUILD_FPGA_
	/* Enable AODMIC clock */
	tegra_clk_enable(ctlr->id.clk);
	tegra_clk_reset_pulse(ctlr->id.rst, 2);
#endif

	/* Read control register */
	ctrl_reg = aodmic_readl(ctlr, AODMIC_CTRL_0);

	/* Setup channel configuration in control register */
	switch (config->channel_config) {
	case TEGRA_AODMIC_CHANNEL_MONO_LEFT:
		num_channels = 1;
		ctrl_reg =
		    NV_FLD_SET_DRF_DEF(AODMIC, CTRL,
		                       CHANNEL_SELECT, LEFT, ctrl_reg);
		break;
	case TEGRA_AODMIC_CHANNEL_MONO_RIGHT:
		num_channels = 1;
		ctrl_reg =
		    NV_FLD_SET_DRF_DEF(AODMIC, CTRL,
		                       CHANNEL_SELECT, RIGHT, ctrl_reg);
		break;
	case TEGRA_AODMIC_CHANNEL_STEREO:
		num_channels = 2;
		ctrl_reg =
		    NV_FLD_SET_DRF_DEF(AODMIC, CTRL,
		                       CHANNEL_SELECT, STEREO, ctrl_reg);
		break;

	default:
		error_hook("Invalid channel configuration");
		ret_val = TEGRA_AODMIC_ERR_INVALID_CHANNELS;
		goto err_exit;
	}
	ctlr->num_channels = num_channels;
	aodmic_writel(ctlr, ctrl_reg, AODMIC_CTRL_0);

	/* Read sample width configuration */
	switch (config->sample_width) {
	case TEGRA_AODMIC_BITS_PER_SAMPLE_16:    samp_bytes = 2; break;
	case TEGRA_AODMIC_BITS_PER_SAMPLE_32:    samp_bytes = 4; break;

	default:
		error_hook("Invalid sample width configuration");
		ret_val = TEGRA_AODMIC_ERR_INVALID_SAMPLE_WIDTH;
		goto err_exit;
	}
	ctlr->samp_bytes = samp_bytes;

	/* Setup interrupt threshold in FIFO control register */
	fifo_ctrl_reg = NV_DRF_NUM(AODMIC, APB_FIFO_CTRL, THRESHOLD,
	                           TEGRA_AODMIC_FIFO_THRESHOLD);
	aodmic_writel(ctlr, fifo_ctrl_reg, AODMIC_APB_FIFO_CTRL_0);

	/* SC filter is enabled by default, enable DCR filter here */
	dbg_ctrl_reg = aodmic_readl(ctlr, AODMIC_DBG_CTRL_0);
	dbg_ctrl_reg = NV_FLD_SET_DRF_DEF(AODMIC, DBG_CTRL,
	                                  DCR_ENABLE, ENABLE, dbg_ctrl_reg);
	aodmic_writel(ctlr, dbg_ctrl_reg, AODMIC_DBG_CTRL_0);

	/* Allocate driver buffer */
	if (((config->period_size % TEGRA_AODMIC_FIFO_THRESHOLD) != 0) ||
	    (config->num_periods < 2)) {
		error_hook("Invalid period configuration");
			ret_val = TEGRA_AODMIC_ERR_INVALID_PERIOD;
			goto err_exit;
	}
	/* Driver buffer may be viewed as a queue of    */
	/* num_periods buffers each of size period_size */
	period_bytes = config->period_size *
	               num_channels * TEGRA_AODMIC_WORD_SIZE;
	ctlr->data   = malloc(period_bytes * config->num_periods);
	if (ctlr->data == NULL) {
		error_hook("Unable to allocate driver buffer");
		ret_val = TEGRA_AODMIC_ERR_MEM_ALLOC_FAILURE;
		goto err_exit;
	}
	ctlr->period_bytes = period_bytes;
	ctlr->num_periods  = config->num_periods;

	/* Create semaphore for tracking free buffers */
	ctlr->xSemaphore_free_buf = xSemaphoreCreateCounting(
	                              ctlr->num_periods /* uxMaxCount */,
	                              0                 /* uxInitialCount */);
	if (ctlr->xSemaphore_free_buf == NULL) {
		error_hook("Unable to create free_buf semaphore");
		ret_val = TEGRA_AODMIC_ERR_SEM_CREATE_FAILURE;
		goto err_exit;
	}

	/* Create queue for tracking filled buffers */
	ctlr->xQueue_filled_buf = xQueueCreate(
	                                ctlr->num_periods /* uxQueueLength */,
	                                sizeof(void*)     /* uxItemSize */);
	if (ctlr->xQueue_filled_buf == NULL) {
		error_hook("Unable to create filled_buf semaphore");
		ret_val = TEGRA_AODMIC_ERR_SEM_CREATE_FAILURE;
		goto err_exit;
	}

	/* Set timeout at queue receive to total buffer duration */
	ctlr->max_read_timeout =
	    ((config->period_size * ctlr->num_periods * 1000) / sample_rate) /
	    portTICK_PERIOD_MS;

	/* GPCDMA contoller ID and channel for transfer from AODMIC FIFO */
	ctlr->gpcdma_id    = config->gpcdma_id;
	ctlr->dma_chan_num = config->dma_chan_num;
	/*$$  IT IS ASSUMED THAT THE GPCDMA CONTROLLER AND THE SELECTED  $$*/
	/*$$  CHANNEL ARE INITIALIZED PRIOR TO CALLING THIS FUNCTION     $$*/

	/* Setup DMA transfer parameters */
	ctlr->dma_xfer.direction      = TEGRA_GPCDMA_XFER_DIR_IO_TO_MEM;
	ctlr->dma_xfer.bus_width      = TEGRA_GPCDMA_IO_BUS_WIDTH_32;
	ctlr->dma_xfer.burst_size     = TEGRA_AODMIC_FIFO_THRESHOLD;
	ctlr->dma_xfer.src_addr       = (void *)(ctlr->id.base_addr +
	                                         AODMIC_APB_FIFO_CTRL_RD_DATA_0);
	ctlr->dma_xfer.src_addr_wrap  = 1;
	ctlr->dma_xfer.dst_addr       = NULL; /* To be filled later */
	ctlr->dma_xfer.dst_addr_wrap  = 0;
	ctlr->dma_xfer.xfer_count     = period_bytes;
	ctlr->dma_xfer.en_flow_ctrl   = true;
	ctlr->dma_xfer.slave_req      = GPCDMA_AO_CHANNEL_CH0_CSR_0_REQ_SEL_DMIC;
	ctlr->dma_xfer.synchronous    = false;
	ctlr->dma_xfer.timeout        = 0;
	ctlr->dma_xfer.callback       = tegra_aodmic_gpcdma_callback;
	ctlr->dma_xfer.callback_param = ctlr;

	/* Enable AODMIC */
	enable_reg = NV_DRF_DEF(AODMIC, ENABLE, ENABLE, TRUE);
	aodmic_writel(ctlr, enable_reg, AODMIC_ENABLE_0);

	/* Return success */
	ctlr->dma_running = 0;
	ctlr->init_done   = 1;
	return 0;

err_exit:

	/* Release all resources and return error */
	tegra_aodmic_close(id);
	return ret_val;
}

void tegra_aodmic_close(struct tegra_aodmic_id *id)
{
	struct tegra_aodmic_ctlr *ctlr = (struct tegra_aodmic_ctlr *)id;
	uint32_t reg_val;

	if (ctlr->init_done) {
		ctlr->init_done = 0;

		/* Abort any DMA transfer in progress */
		tegra_gpcdma_abort(ctlr->gpcdma_id, ctlr->dma_chan_num);

		/* Reset AODMIC */
		reg_val = NV_DRF_DEF(AODMIC, SOFT_RESET, SOFT_RESET, TRUE);
		aodmic_writel(ctlr, reg_val, AODMIC_SOFT_RESET_0);

		/* Disable AODMIC */
		reg_val = NV_DRF_DEF(AODMIC, ENABLE, ENABLE, FALSE);
		aodmic_writel(ctlr, reg_val, AODMIC_ENABLE_0);

		/* Disable AODMIC clock */
		tegra_clk_disable(ctlr->id.clk);
	}

	/* Release resources that may have been acquired */
	if (ctlr->xQueue_filled_buf != NULL) {
		vQueueDelete(ctlr->xQueue_filled_buf);
		ctlr->xQueue_filled_buf = NULL;
	}
	if (ctlr->xSemaphore_free_buf != NULL) {
		vSemaphoreDelete(ctlr->xSemaphore_free_buf);
		ctlr->xSemaphore_free_buf = NULL;
	}
	if (ctlr->data != NULL) {
		free(ctlr->data);
		ctlr->data = NULL;
	}
}

int tegra_aodmic_read(struct tegra_aodmic_id *id, void *data, uint32_t count)
{
	struct tegra_aodmic_ctlr *ctlr = (struct tegra_aodmic_ctlr *)id;
	enum dma_status dma_ret;
	BaseType_t status;
	int32_t i;
	uint8_t *write_buf;
	int32_t pcm_samp;

	if (!(ctlr->init_done)) {
		error_hook("AODMIC not initialized");
		return TEGRA_AODMIC_ERR_NOT_INITIALIZED;
	}

	if (data == NULL) {
		error_hook("Invalid buffer");
		return TEGRA_AODMIC_ERR_BUF_INVALID;
	}
	write_buf = data;

	/* Trigger DMA if it is in halted state; halted state */
	/* may be at init or due to buffer overflow           */
	if (!(ctlr->dma_running)) {
		/* Drain out any buffers that may already be filled */
		xQueueReset(ctlr->xQueue_filled_buf);
		ctlr->read_ptr      = NULL;
		ctlr->bytes_to_read = 0;

		/* Setup all buffers as free (for DMA to write) */
		do {
			status = xSemaphoreGive(ctlr->xSemaphore_free_buf);
		} while (status);
		ctlr->free_pos = 0;

		/* Trigger first DMA transfer; subsequent transfers */
		/* will be triggered from the DMA ISR callback      */
		ctlr->dma_running = 1;
		xSemaphoreTake(ctlr->xSemaphore_free_buf, 0);
		ctlr->dma_xfer.dst_addr = ctlr->data;
		dma_ret = tegra_gpcdma_transfer(ctlr->gpcdma_id,
		                                ctlr->dma_chan_num,
		                                &ctlr->dma_xfer);
		if (dma_ret != DMA_STATUS_EXECUTING) {
			error_hook("Error in tegra_gpcdma_transfer");
			ctlr->dma_running = 0;
			return TEGRA_AODMIC_ERR_DMA_XFER_ERROR;
		}
	}

	/* Read PCM data from filled buffers */
	for (i = 0; i < count; i += ctlr->samp_bytes) {
		/* If current buffer is exhausted, proceed to next buffer */
		if (ctlr->bytes_to_read == 0) {
			/* Release consumed buffer for writing */
			if (ctlr->read_ptr != NULL)
				xSemaphoreGive(ctlr->xSemaphore_free_buf);

			/* Get next buffer from filled queue */
			ctlr->read_ptr = NULL;
			status = xQueueReceive(ctlr->xQueue_filled_buf,
			                       &ctlr->read_ptr,
			                       ctlr->max_read_timeout);
			if (!status) {
				error_hook("Timeout at DMA transfer");
				return TEGRA_AODMIC_ERR_READ_TIMEOUT;
			}
			ctlr->bytes_to_read = ctlr->period_bytes;
		}

		/* AODMIC provides 24-bit PCM packed in 32 bits,  */
		/* Little Endian, aligned to LSB (i.e. bits 23:0) */
		pcm_samp             = (*((int32_t *)(ctlr->read_ptr))) << 8;
		ctlr->read_ptr      += TEGRA_AODMIC_WORD_SIZE;
		ctlr->bytes_to_read -= TEGRA_AODMIC_WORD_SIZE;

		/* Format PCM sample to requested width */
		switch (ctlr->samp_bytes) {
		case 2:
			*((int16_t *)(write_buf + i)) = (int16_t)
			                                (pcm_samp >> 16);
			break;
		case 4:
			*((int32_t *)(write_buf + i)) = pcm_samp;
			break;
		}
	}

	return 0;
}
