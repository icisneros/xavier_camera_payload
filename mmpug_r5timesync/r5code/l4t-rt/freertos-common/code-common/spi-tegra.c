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
#include <stdlib.h>
#include <string.h>

#include <FreeRTOS.h>
#include <task.h>

#include <arspi.h>
#include <nvrm_drf.h>

#include <delay.h>
#include <spi-tegra-priv.h>
#include <irqs.h>
#include <macros.h>
#include <reg-access.h>
#include <err-hook.h>
#include <printf-isr.h>
#include <cache.h>
#include <cache-hw.h>
#include <ccplex-cache-hw.h>

/* Too much of info with VDEBUG.
 * Enable to get analyzer like data
 * when probing external device is not possible
 */
#ifdef ENABLE_VDEBUG
#define vdbgprintf printf
#else
static inline void vdbgprintf(const char *fmt, ...)
{
}
#endif

#ifdef ENABLE_DEBUG
#define dbgprintf	printf
#define dbgprintf_isr	printf_isr
#else
static inline void dbgprintf(const char *fmt, ...)
{
}
static inline void dbgprintf_isr(const char *fmt, ...)
{
}
#endif

#define SPI_ERR			(NV_DRF_DEF(SPI, FIFO_STATUS, ERR, ERROR))
#define SPI_FIFO_ERROR		(NV_DRF_DEF(SPI, FIFO_STATUS, RX_FIFO_UNF, \
					ERROR) | \
				NV_DRF_DEF(SPI, FIFO_STATUS, RX_FIFO_OVF, \
					ERROR) | \
				NV_DRF_DEF(SPI, FIFO_STATUS, TX_FIFO_UNF, \
					ERROR) | \
				NV_DRF_DEF(SPI, FIFO_STATUS, RX_FIFO_OVF, \
					ERROR))
#define SPI_FIFO_EMPTY		(NV_DRF_DEF(SPI, FIFO_STATUS, RX_FIFO_EMPTY, \
					EMPTY) | \
				NV_DRF_DEF(SPI, FIFO_STATUS, TX_FIFO_EMPTY, \
					EMPTY))

#define SPI_FIFO_DEPTH		64
#define DATA_DIR_TX		(1 << 0)
#define DATA_DIR_RX		(1 << 1)

#define SPI_FIFO_FLUSH_MAX_DELAY	2000

#define SPI_SPEED_TAP_DELAY_MARGIN	35000000
#define SPI_DEFAULT_RX_TAP_DELAY	10

#define SPI_TRANSFER_TIMEOUT	(3000 /* ms */ / portTICK_PERIOD_MS)

#define CACHE_LINE_SZ	max(CCPLEX_CACHE_LINE_SIZE, CACHE_LINE_SIZE)
#define CACHE_LINE_MASK (~(CACHE_LINE_SZ - 1))

static inline uint32_t tegra_spi_readl(struct tegra_spi_ctlr *tspi,
					uint32_t reg)
{
	return readl(tspi->conf.base_addr + reg);
}

static inline void tegra_spi_writel(struct tegra_spi_ctlr *tspi,
					uint32_t val, uint32_t reg)
{
	writel(val, tspi->conf.base_addr + reg);
}

void tegra_spi_dma_complete(void *callback_param, enum dma_status status);

static void tegra_spi_clear_status(struct tegra_spi_ctlr *tspi)
{
	uint32_t val;

	/* Write 1 to clear status register */
	val = tegra_spi_readl(tspi, SPI_TRANSFER_STATUS_0);
	tegra_spi_writel(tspi, val, SPI_TRANSFER_STATUS_0);

	val = tegra_spi_readl(tspi, SPI_INTR_MASK_0);
	if (!(val & NV_DRF_DEF(SPI, INTR_MASK, RDY_INTR_MASK, DEFAULT_MASK))) {
		val |= (NV_DRF_DEF(SPI, INTR_MASK, RDY_INTR_MASK,
				DEFAULT_MASK) |
			NV_DRF_DEF(SPI, INTR_MASK, RX_FIFO_UNF_INTR_MASK,
				DEFAULT_MASK) |
			NV_DRF_DEF(SPI, INTR_MASK, TX_FIFO_UNF_INTR_MASK,
				DEFAULT_MASK) |
			NV_DRF_DEF(SPI, INTR_MASK, RX_FIFO_OVF_INTR_MASK,
				DEFAULT_MASK) |
			NV_DRF_DEF(SPI, INTR_MASK, TX_FIFO_OVF_INTR_MASK,
				DEFAULT_MASK));
		tegra_spi_writel(tspi, val, SPI_INTR_MASK_0);
	}

	/* Clear FIFO status error if any */
	val = tegra_spi_readl(tspi, SPI_FIFO_STATUS_0);
	if (val & SPI_ERR)
		tegra_spi_writel(tspi, SPI_ERR | SPI_FIFO_ERROR,
					SPI_FIFO_STATUS_0);
}

static uint32_t tegra_spi_calculate_curr_xfer_param(struct tegra_spi_ctlr *tspi,
						struct tegra_spi_xfer *xfer)
{
	uint32_t remain_len = xfer->len - tspi->cur_pos;
	uint32_t max_word;
	uint32_t max_len;
	uint32_t total_fifo_words;
	bool spi_no_dma = tspi->cdata[xfer->chip_select].spi_no_dma;

	dbgprintf("inside %s\r\n", __func__);
	tspi->bytes_per_word = (xfer->bits_per_word - 1) / 8 + 1;
	/*
	 * SPI transfer length should be multiple of SPI word size
	 * where SPI word size should be power-of-two multiple
	 */
	if (tspi->bytes_per_word == 3)
		tspi->bytes_per_word = 4;

	if ((xfer->bits_per_word == 8 || xfer->bits_per_word == 16 ||
			xfer->bits_per_word == 32) && (remain_len > 3)) {
		tspi->is_packed = 1;
		tspi->words_per_32bit = 32 / xfer->bits_per_word;
	} else {
		tspi->is_packed = 0;
		tspi->words_per_32bit = 1;
	}

	if (tspi->is_packed) {
		max_len = remain_len;
		if (spi_no_dma)
			max_len = min(max_len, (SPI_FIFO_DEPTH * 4));
		tspi->curr_dma_words = max_len / tspi->bytes_per_word;
		total_fifo_words = (max_len + 3) / 4;
	} else {
		max_word = (remain_len - 1) / tspi->bytes_per_word + 1;
		if (spi_no_dma)
			max_word = min(max_word, SPI_FIFO_DEPTH);
		tspi->curr_dma_words = max_word;
		total_fifo_words = max_word;
	}

	dbgprintf("len = %d pos = %d remain_len = %d dma_words = %d \
			bypw = %d\r\n", xfer->len, (int)tspi->cur_pos,
			(int)remain_len, (int)tspi->curr_dma_words,
			(int)tspi->bytes_per_word);
	return total_fifo_words;
}

static uint32_t tegra_spi_fill_tx_fifo_from_client_txbuf(
						struct tegra_spi_ctlr *tspi,
						struct tegra_spi_xfer *xfer)
{
	uint32_t nbytes;
	uint32_t tx_empty_count;
	uint32_t fifo_status;
	uint32_t max_n_32bit;
	uint32_t i, count;
	uint32_t x;
	uint32_t written_words;
	uint32_t fifo_words_left;
	uint8_t *tx_buf = (uint8_t *)xfer->tx_buf + tspi->cur_tx_pos;

	dbgprintf("inside %s\r\n", __func__);
	fifo_status = tegra_spi_readl(tspi, SPI_FIFO_STATUS_0);
	tx_empty_count = NV_DRF_VAL(SPI, FIFO_STATUS, TX_FIFO_EMPTY_COUNT,
					fifo_status);

	vdbgprintf("Tx FIFO Packed=%d\r\n", tspi->is_packed);
	if (tspi->is_packed) {
		fifo_words_left = tx_empty_count * tspi->words_per_32bit;
		written_words = min(fifo_words_left, tspi->curr_dma_words);
		nbytes = written_words * tspi->bytes_per_word;
		max_n_32bit = DIV_ROUND_UP(nbytes, 4);
		for (count = 0; count < max_n_32bit; count++) {
			x = 0;
			for (i = 0; (i < 4) && nbytes; i++, nbytes--)
				x |= ((*tx_buf++) << (i * 8));
			vdbgprintf("X[%d]=%x\r\t", (int)count, (int)x);
			tegra_spi_writel(tspi, x, SPI_TX_FIFO_0);
		}
	} else {
		max_n_32bit = min(tspi->curr_dma_words, tx_empty_count);
		written_words = max_n_32bit;
		nbytes = written_words * tspi->bytes_per_word;
		for (count = 0; count < max_n_32bit; count++) {
			x = 0;
			for (i = 0; nbytes && (i < tspi->bytes_per_word);
							i++, nbytes--)
				x |= ((*tx_buf++) << (i * 8));
			tegra_spi_writel(tspi, x, SPI_TX_FIFO_0);
			vdbgprintf("X[%d]=%x \r\n", (int)count, (int)x);
		}
	}
	tspi->cur_tx_pos += written_words * tspi->bytes_per_word;
	vdbgprintf(" done\r\n");

	return written_words;
}

static int tegra_spi_read_rx_fifo_to_client_rxbuf(struct tegra_spi_ctlr *tspi,
						struct tegra_spi_xfer *xfer)
{
	uint32_t rx_full_count;
	uint32_t fifo_status;
	uint32_t i, count;
	uint32_t x;
	int read_words = 0;
	uint32_t len;
	uint8_t *rx_buf = (uint8_t *)xfer->rx_buf + tspi->cur_rx_pos;

	dbgprintf("inside %s\r\n", __func__);
	fifo_status = tegra_spi_readl(tspi, SPI_FIFO_STATUS_0);
	rx_full_count = NV_DRF_VAL(SPI, FIFO_STATUS, RX_FIFO_FULL_COUNT,
					fifo_status);

	vdbgprintf("RX full count=%u RX FIFO Packed=%d\r\n",
						(unsigned int)rx_full_count,
						tspi->is_packed);
	if (tspi->is_packed) {
		len = tspi->curr_dma_words * tspi->bytes_per_word;
		for (count = 0; count < rx_full_count; count++) {
			x = tegra_spi_readl(tspi, SPI_RX_FIFO_0);
			for (i = 0; len && (i < 4); i++, len--)
				*rx_buf++ = (x >> (i * 8)) & 0xFF;
			vdbgprintf("X[%d]=%x \r\n", (int)count, (int)x);
		}
		tspi->cur_rx_pos += tspi->curr_dma_words * tspi->bytes_per_word;
		read_words += tspi->curr_dma_words;
	} else {
		for (count = 0; count < rx_full_count; count++) {
			x = tegra_spi_readl(tspi, SPI_RX_FIFO_0);
			for (i = 0; (i < tspi->bytes_per_word); i++)
				*rx_buf++ = (x >> (i * 8)) & 0xFF;
			vdbgprintf("X[%d]=%x \r\n", (int)count, (int)x);
		}
		tspi->cur_rx_pos += rx_full_count * tspi->bytes_per_word;
		read_words += rx_full_count;
	}

	return read_words;
}

static void tegra_spi_copy_client_txbuf_to_spi_txbuf(
						struct tegra_spi_ctlr *tspi,
						struct tegra_spi_xfer *xfer)
{
	unsigned int i;
	int count;
	unsigned int x;
	unsigned int offset;
	unsigned char *tx_c_buf;
	uint32_t base;
	uint32_t *tx_buf = (uint32_t *)((uint8_t *)xfer->tx_buf + tspi->cur_tx_pos);
	uint32_t nbytes = tspi->curr_dma_words * tspi->bytes_per_word;

	dbgprintf("inside %s\r\n", __func__);
	vdbgprintf("TX DMA:: Packed = %d tx_pos = %d\r\n", tspi->is_packed,
							(int)tspi->cur_tx_pos);
	if (!tspi->is_packed) {
		tx_c_buf = (unsigned char *)xfer->tx_buf + tspi->cur_tx_pos +
						nbytes - 1;
		/* curr_dma_words and fifo_words are same for unpacked mode */
		for (count = tspi->curr_dma_words - 1; count >= 0; count--) {
			x = 0;
			for (i = 0; nbytes && (i < tspi->bytes_per_word);
						i++) {
				offset = (--nbytes) % (tspi->bytes_per_word);
				x |= (*tx_c_buf--) << (offset * 8);
			}
			vdbgprintf("X[%d]=%x ", (int)count, x);
			tx_buf[count] = x;
		}
		/**
		 * The following cache flush is needed only for unpacked mode
		 * as for packed mode you do not need to modify the tx buffer
		 */
		if (xfer->flags & BIT(TEGRA_SPI_XFER_HANDLE_CACHE)) {
			/**
			 * Align buffer address and length to cache line size
			 * and flush the cache
			 */
			base = (uint32_t) xfer->tx_buf & CACHE_LINE_MASK;
			cache_clean((void *)base, (((tspi->curr_dma_words * 4) +
					(CACHE_LINE_SZ - 1)) & CACHE_LINE_MASK));
		}
	}
	vdbgprintf("\r\n");
}

static void tegra_spi_copy_spi_rxbuf_to_client_rxbuf(
						struct tegra_spi_ctlr *tspi,
						struct tegra_spi_xfer *xfer)
{
	unsigned int i;
	unsigned int count;
	unsigned int x;
	unsigned int rx_mask;
	uint32_t base;
	uint32_t *rx_buf = (uint32_t *)((uint8_t *)xfer->rx_buf +
						tspi->cur_rx_pos);
	uint8_t *rx_c_buf = (uint8_t *)xfer->rx_buf +
					tspi->cur_rx_pos;

	dbgprintf("inside %s\r\n", __func__);
	vdbgprintf("RX DMA:: Packed=%d rx_pos = %d\r\n", tspi->is_packed,
						(int)tspi->cur_rx_pos);
	/**
	 * Align buffer address and length to cache line size
	 * and invalidate the cache
	 */
	if (xfer->flags & BIT(TEGRA_SPI_XFER_HANDLE_CACHE)) {
		base = (uint32_t) xfer->rx_buf & CACHE_LINE_MASK;
		cache_invalidate((void *)base,
				((xfer->len + (CACHE_LINE_SZ - 1)) &
				CACHE_LINE_MASK));
	}
	if (!tspi->is_packed) {
		rx_mask = ((uint64_t)1 << xfer->bits_per_word) - 1;
		for (count = 0; count < tspi->curr_dma_words; count++) {
			x = rx_buf[count];
			x &= rx_mask;
			for (i = 0; (i < tspi->bytes_per_word); i++)
				*rx_c_buf++ = (x >> (i*8)) & 0xFF;
			vdbgprintf("X[%d]=%x\r\t", (int)count, (int)x);
		}
	}
	vdbgprintf("\r\n");
	tspi->cur_rx_pos += tspi->curr_dma_words * tspi->bytes_per_word;
}

void tegra_spi_dma_complete(void *callback_param, enum dma_status status)
{
	struct tegra_spi_ctlr *tspi = (struct tegra_spi_ctlr *)callback_param;
	BaseType_t higher_prio_task_woken = pdFALSE;

	tspi->dma_status = status;

	if (status != DMA_STATUS_COMPLETE)
		error_hook("SPI: abnormal DMA transfer");
	else
		xQueueSendToBackFromISR(tspi->dma_queue, &tspi->dma_status,
					&higher_prio_task_woken);

	portYIELD_FROM_ISR(higher_prio_task_woken);
}
static void tegra_spi_dump_dma(struct tegra_gpcdma_xfer *dmaxfer)
{
	vdbgprintf(":::::DMA PARAMS:::::\r\n");
	vdbgprintf("direction = %d\r\n", dmaxfer->direction);
	vdbgprintf("bus_width = %d\r\n", dmaxfer->bus_width);
	vdbgprintf("burst_size = %d\r\n", dmaxfer->burst_size);
	vdbgprintf("src_addr = %x\r\n", (unsigned int)dmaxfer->src_addr);
	vdbgprintf("src_addr_wrap = %d\r\n", dmaxfer->src_addr_wrap);
	vdbgprintf("dst_addr = %x\r\n", (unsigned int)dmaxfer->dst_addr);
	vdbgprintf("dst_addr_wrap = %d\r\n", dmaxfer->dst_addr_wrap);
	vdbgprintf("xfer_count = %d\r\n", (int)dmaxfer->xfer_count);
	vdbgprintf("en_flow_ctrl = %d\r\n", dmaxfer->en_flow_ctrl);
	vdbgprintf("slave_req = %d\r\n", dmaxfer->slave_req);
	vdbgprintf("synchronous = %d\r\n", dmaxfer->synchronous);
	vdbgprintf("timeout = %d\r\n", (int)dmaxfer->timeout);
}

static int tegra_spi_start_tx_dma(struct tegra_spi_ctlr *tspi,
				struct tegra_spi_xfer *xfer,
				uint32_t len, uint32_t burst_size)
{
	uint8_t *tx_buf = (uint8_t *)xfer->tx_buf + tspi->cur_tx_pos;

	dbgprintf("inside %s\r\n", __func__);
	struct tegra_gpcdma_xfer dma_xfer = {
		.direction = TEGRA_GPCDMA_XFER_DIR_MEM_TO_IO,
		.bus_width = TEGRA_GPCDMA_IO_BUS_WIDTH_32,
		.burst_size = burst_size,
		.src_addr = tx_buf,
		.src_addr_wrap = 0,
		.dst_addr = (void *)(tspi->conf.base_addr + SPI_TX_FIFO_0),
		.dst_addr_wrap = 1,
		.xfer_count = len,
		.en_flow_ctrl = true,
		.slave_req = tspi->dma_slave_req,
		.synchronous = false,
		.timeout = 0,
		.callback = tegra_spi_dma_complete,
		.callback_param = tspi,
	};
	tegra_spi_dump_dma(&dma_xfer);

	return tegra_gpcdma_transfer(tspi->dma_id, tspi->dma_channel.tx,
					&dma_xfer);
}

static int tegra_spi_start_rx_dma(struct tegra_spi_ctlr *tspi,
				struct tegra_spi_xfer *xfer,
				uint32_t len, uint32_t burst_size)
{
	uint8_t *rx_buf = (uint8_t *)xfer->rx_buf + tspi->cur_rx_pos;

	dbgprintf("inside %s\r\n", __func__);
	struct tegra_gpcdma_xfer dma_xfer = {
		.direction = TEGRA_GPCDMA_XFER_DIR_IO_TO_MEM,
		.bus_width = TEGRA_GPCDMA_IO_BUS_WIDTH_32,
		.burst_size = burst_size,
		.src_addr = (void *)(tspi->conf.base_addr + SPI_RX_FIFO_0),
		.src_addr_wrap = 1,
		.dst_addr = rx_buf,
		.dst_addr_wrap = 0,
		.xfer_count = len,
		.en_flow_ctrl = true,
		.slave_req = tspi->dma_slave_req,
		.synchronous = false,
		.timeout = 0,
		.callback = tegra_spi_dma_complete,
		.callback_param = tspi,
	};
	tegra_spi_dump_dma(&dma_xfer);

	return tegra_gpcdma_transfer(tspi->dma_id, tspi->dma_channel.rx,
					&dma_xfer);
}

static int check_and_clear_fifo(struct tegra_spi_ctlr *tspi)
{
	unsigned long status;
	int cnt = SPI_FIFO_FLUSH_MAX_DELAY;

	/* Make sure that Rx and Tx FIFO are empty */
	status = tegra_spi_readl(tspi, SPI_FIFO_STATUS_0);
	if ((status & SPI_FIFO_EMPTY) != SPI_FIFO_EMPTY) {
		/* flush the FIFO */
		status = NV_FLD_SET_DRF_DEF(SPI, FIFO_STATUS,
						RX_FIFO_FLUSH, FLUSH, status);
		status = NV_FLD_SET_DRF_DEF(SPI, FIFO_STATUS,
						TX_FIFO_FLUSH, FLUSH, status);
		tegra_spi_writel(tspi, status, SPI_FIFO_STATUS_0);
		do {
			status = tegra_spi_readl(tspi, SPI_FIFO_STATUS_0);
			if ((status & SPI_FIFO_EMPTY) == SPI_FIFO_EMPTY)
				return 0;
			udelay(1);
		} while (cnt--);
		error_hook("SPI: RX/TX FIFO are not empty");
		return -1;
	}

	return 0;
}

static int tegra_spi_start_dma_based_transfer(
		struct tegra_spi_ctlr *tspi, struct tegra_spi_xfer *xfer)
{
	uint32_t val = 0, cmd1;
	uint32_t intr_mask;
	uint32_t len;
	uint32_t burst_size;
	int ret = 0;

	dbgprintf("SPI: dma transfer\r\n");

	/* For half duplex, ensure rx and tx transfers are not enabled.
	 * If rx and tx transfers are enabled, return error.
	 */
	if (tspi->dma_channel.tx == tspi->dma_channel.rx) {
		if ((tspi->cur_direction & DATA_DIR_TX) &&
				(tspi->cur_direction & DATA_DIR_RX)) {
			error_hook("SPI: Full duplex xfer with single dma ch");
			return -1;
		}
	}
	val = NV_FLD_SET_DRF_NUM(SPI, DMA_BLK_SIZE, DMA_BLOCK_SIZE,
				(tspi->curr_dma_words - 1), val);
	tegra_spi_writel(tspi, val, SPI_DMA_BLK_SIZE_0);

	if (tspi->is_packed)
		len = DIV_ROUND_UP(tspi->curr_dma_words * tspi->bytes_per_word,
				 4) * 4;
	else
		len = tspi->curr_dma_words * 4;

	/* Set attention level based on length of transfer.
	 * Currently, APBDMA does not support trigger level 16.
	 * DMA burst size and SPI trigger level should be same.
	 */
	val = 0;
	if (len & 0xF) {
		val |= NV_DRF_DEF(SPI, DMA_CTL, TX_TRIG, TRIG1) |
			NV_DRF_DEF(SPI, DMA_CTL, RX_TRIG, TRIG1);
		burst_size = 1;
	} else if (((len) >> 4) & 0x1) {
		val |= NV_DRF_DEF(SPI, DMA_CTL, TX_TRIG, TRIG4) |
			NV_DRF_DEF(SPI, DMA_CTL, RX_TRIG, TRIG4);
		burst_size = 4;
	} else {
		val |= NV_DRF_DEF(SPI, DMA_CTL, TX_TRIG, TRIG8) |
			NV_DRF_DEF(SPI, DMA_CTL, RX_TRIG, TRIG8);
		burst_size = 8;
	}

	if ((tspi->cur_direction & DATA_DIR_TX) ||
			(tspi->cur_direction & DATA_DIR_RX)) {
		intr_mask = tegra_spi_readl(tspi, SPI_INTR_MASK_0);
		intr_mask &= ~(NV_DRF_DEF(SPI, INTR_MASK,
					RDY_INTR_MASK, DEFAULT_MASK) |
				NV_DRF_DEF(SPI, INTR_MASK,
					RX_FIFO_UNF_INTR_MASK, DEFAULT_MASK) |
				NV_DRF_DEF(SPI, INTR_MASK,
					TX_FIFO_UNF_INTR_MASK, DEFAULT_MASK) |
				NV_DRF_DEF(SPI, INTR_MASK,
					RX_FIFO_OVF_INTR_MASK, DEFAULT_MASK) |
				NV_DRF_DEF(SPI, INTR_MASK,
					TX_FIFO_OVF_INTR_MASK, DEFAULT_MASK));
		tegra_spi_writel(tspi, intr_mask, SPI_INTR_MASK_0);
	}

	tegra_spi_writel(tspi, val, SPI_DMA_CTL_0);
	tspi->dma_ctrl_reg_val = val;

	if (tspi->cur_direction & DATA_DIR_TX) {
		tegra_spi_copy_client_txbuf_to_spi_txbuf(tspi, xfer);
		ret = tegra_spi_start_tx_dma(tspi, xfer, len, burst_size);
		if (ret < 0) {
			error_hook("SPI: Starting TX DMA failed");
			return ret;
		}
	}
	if (tspi->cur_direction & DATA_DIR_RX) {
		ret = tegra_spi_start_rx_dma(tspi, xfer, len, burst_size);
		if (ret < 0) {
			error_hook("SPI: Starting RX DMA failed");
			return ret;
		}
	}

	tspi->is_curr_dma_xfer = true;
	cmd1 = tegra_spi_readl(tspi, SPI_COMMAND_0);
	if (tspi->cur_direction & DATA_DIR_TX)
		cmd1 |= NV_DRF_DEF(SPI, COMMAND, Tx_EN, ENABLE);
	if (tspi->cur_direction & DATA_DIR_RX)
		cmd1 |= NV_DRF_DEF(SPI, COMMAND, Rx_EN, ENABLE);
	tegra_spi_writel(tspi, cmd1, SPI_COMMAND_0);

	val |= NV_DRF_DEF(SPI, DMA_CTL, DMA_EN, ENABLE);
	tegra_spi_writel(tspi, val, SPI_DMA_CTL_0);

	return ret;
}

static int tegra_spi_start_cpu_based_transfer(
		struct tegra_spi_ctlr *tspi, struct tegra_spi_xfer *xfer)
{
	uint32_t val = 0;
	uint32_t intr_mask;
	uint32_t cur_words;

	dbgprintf("SPI: cpu transfer\r\n");
	if (tspi->cur_direction & DATA_DIR_TX)
		cur_words = tegra_spi_fill_tx_fifo_from_client_txbuf(tspi,
								xfer);
	else
		cur_words = tspi->curr_dma_words;

	val = NV_FLD_SET_DRF_NUM(SPI, DMA_BLK_SIZE, DMA_BLOCK_SIZE,
					(cur_words - 1), val);

	tegra_spi_writel(tspi, val, SPI_DMA_BLK_SIZE_0);

	if ((tspi->cur_direction & DATA_DIR_TX) ||
			(tspi->cur_direction & DATA_DIR_RX)) {
		intr_mask = tegra_spi_readl(tspi, SPI_INTR_MASK_0);
		intr_mask &= ~(NV_DRF_DEF(SPI, INTR_MASK, RDY_INTR_MASK,
				DEFAULT_MASK) |
				NV_DRF_DEF(SPI, INTR_MASK,
					RX_FIFO_UNF_INTR_MASK, DEFAULT_MASK) |
				NV_DRF_DEF(SPI, INTR_MASK,
					TX_FIFO_UNF_INTR_MASK, DEFAULT_MASK) |
				NV_DRF_DEF(SPI, INTR_MASK,
					RX_FIFO_OVF_INTR_MASK, DEFAULT_MASK) |
				NV_DRF_DEF(SPI, INTR_MASK,
					TX_FIFO_OVF_INTR_MASK, DEFAULT_MASK));
		tegra_spi_writel(tspi, intr_mask, SPI_INTR_MASK_0);
	}

	val = 0;
	tegra_spi_writel(tspi, val, SPI_DMA_CTL_0);
	tspi->dma_ctrl_reg_val = val;

	tspi->is_curr_dma_xfer = false;
	val = tspi->cmd1_reg_val;
	if (tspi->cur_direction & DATA_DIR_TX)
		val |= NV_DRF_DEF(SPI, COMMAND, Tx_EN, ENABLE);
	if (tspi->cur_direction & DATA_DIR_RX)
		val |= NV_DRF_DEF(SPI, COMMAND, Rx_EN, ENABLE);
	tegra_spi_writel(tspi, val, SPI_COMMAND_0);

	dbgprintf("SPI: SPI_COMMAND_0 written 0x%x\r\n",
				(unsigned int)val);
	val = NV_FLD_SET_DRF_DEF(SPI, COMMAND, PIO, PIO, val);
	tegra_spi_writel(tspi, val, SPI_COMMAND_0);

	return 0;
}

static int tegra_spi_clk_set_rate(struct tegra_spi_ctlr *tspi, uint32_t rate)
{
#ifndef _NV_BUILD_FPGA_
	int ret = 0;

	/* TODO: Change parent to support lower frequencies
	 * Currently there is no option to change parent for a clock
	 * minimum supported frequency on PLLP is 3.5MHz
	 */

	dbgprintf("SPI: setting clk rate: %d\r\n", (int)rate);
	ret = tegra_clk_disable(tspi->conf.div_clk);
	if (ret) {
		error_hook("SPI: clock disable failed");
		return -1;
	}
	ret = tegra_clk_set_rate(tspi->conf.div_clk, rate);
	if (ret)
		error_hook("SPI: Failed to set clk freq");

	ret = tegra_clk_enable(tspi->conf.div_clk);
	if (ret) {
		error_hook("SPI: clock enable failed");
		return -1;
	}
#endif
	return 0;
}

int tegra_spi_suspend(struct tegra_spi_id *id)
{
	struct tegra_spi_ctlr *tspi = (struct tegra_spi_ctlr *)id;

	if (tspi->busy)
		return -1;

	tegra_clk_disable(tspi->conf.div_clk);

	return 0;
}

int tegra_spi_resume(struct tegra_spi_id *id)
{
	struct tegra_spi_ctlr *tspi = (struct tegra_spi_ctlr *)id;

	tegra_spi_clk_set_rate(tspi, tspi->spi_clk_rate);

	return 0;
}

static int tegra_spi_start_transfer_one(struct tegra_spi_ctlr *tspi,
					struct tegra_spi_xfer *xfer)
{
	uint32_t total_fifo_words;
	int ret = 0;
	uint32_t command1;
	uint32_t speed;
	uint32_t cs_pol_bit[MAX_CHIP_SELECT] = {
			NV_DRF_DEF(SPI, COMMAND, CS_POL_INACTIVE0, HIGH),
			NV_DRF_DEF(SPI, COMMAND, CS_POL_INACTIVE1, HIGH),
			NV_DRF_DEF(SPI, COMMAND, CS_POL_INACTIVE2, HIGH),
			NV_DRF_DEF(SPI, COMMAND, CS_POL_INACTIVE3, HIGH),
	};

	dbgprintf("SPI: tegra_spi_start_transfer_one\r\n");
	tspi->cur_pos = 0;
	tspi->cur_rx_pos = 0;
	tspi->cur_tx_pos = 0;
	tspi->curr_xfer = xfer;
	tspi->tx_status = 0;
	tspi->rx_status = 0;
	total_fifo_words = tegra_spi_calculate_curr_xfer_param(tspi, xfer);

	/* Check that the all words are available */
	if (xfer->len % tspi->bytes_per_word != 0)
		return 1;
	speed = xfer->spi_clk_rate ? xfer->spi_clk_rate :
			tspi->cdata[xfer->chip_select].spi_max_clk_rate;
	if (speed != tspi->spi_clk_rate) {
		ret = tegra_spi_clk_set_rate(tspi, speed);
		if (ret < 0)
			return ret;
		else
			tspi->spi_clk_rate = speed;
	}

	tegra_spi_clear_status(tspi);

	command1 = tspi->def_cmd1_reg_val;
	if (xfer->mode & TEGRA_SPI_CS_HIGH)
		command1 &= ~cs_pol_bit[xfer->chip_select];
	else
		command1 |= cs_pol_bit[xfer->chip_select];

	tspi->def_cmd1_reg_val = command1;

	command1 = NV_FLD_SET_DRF_NUM(SPI, COMMAND, BIT_LENGTH,
			(xfer->bits_per_word - 1), command1);
	command1 = NV_FLD_SET_DRF_NUM(SPI, COMMAND, MODE, (xfer->mode & 0x3),
					command1);
	command1 = NV_FLD_SET_DRF_DEF(SPI, COMMAND, CS_SW_HW, SOFTWARE,
					command1);
	command1 = NV_FLD_SET_DRF_NUM(SPI, COMMAND, CS_SW_VAL,
					(xfer->mode & TEGRA_SPI_CS_HIGH),
					command1);

	if (xfer->mode & TEGRA_SPI_LSBYTE_FIRST)
		command1 = NV_FLD_SET_DRF_DEF(SPI, COMMAND, En_LE_Byte,
						FIRST, command1);
	else
		command1 = NV_FLD_SET_DRF_DEF(SPI, COMMAND, En_LE_Byte,
						LAST, command1);

	if (xfer->mode & TEGRA_SPI_LSB_FIRST)
		command1 = NV_FLD_SET_DRF_DEF(SPI, COMMAND, En_LE_Bit,
						FIRST, command1);
	else
		command1 = NV_FLD_SET_DRF_DEF(SPI, COMMAND, En_LE_Bit,
						LAST, command1);

	if (xfer->mode & TEGRA_SPI_3WIRE)
		command1 = NV_FLD_SET_DRF_DEF(SPI, COMMAND, BIDIR, BIDIR,
						command1);
	else
		command1 = NV_FLD_SET_DRF_DEF(SPI, COMMAND, BIDIR, NORMAL,
						command1);

	if ((xfer->rx_nbits == TEGRA_SPI_NBITS_DUAL) ||
			(xfer->tx_nbits == TEGRA_SPI_NBITS_DUAL))
		command1 = NV_FLD_SET_DRF_DEF(SPI, COMMAND, BOTH_EN_BIT,
						ENABLE, command1);

	if (tspi->is_packed)
		command1 |= NV_DRF_DEF(SPI, COMMAND, PACKED, ENABLE);
	else
		command1 &= ~(NV_DRF_DEF(SPI, COMMAND, PACKED, ENABLE));

	command1 &= ~(NV_DRF_DEF(SPI, COMMAND, CS_SEL, DEFAULT_MASK) |
			NV_DRF_DEF(SPI, COMMAND, Tx_EN, DEFAULT_MASK) |
			NV_DRF_DEF(SPI, COMMAND, Rx_EN, DEFAULT_MASK));
	tspi->cur_direction = 0;
	if (xfer->rx_buf)
		tspi->cur_direction |= DATA_DIR_RX;
	if (xfer->tx_buf)
		tspi->cur_direction |= DATA_DIR_TX;

	command1 = NV_FLD_SET_DRF_NUM(SPI, COMMAND, CS_SEL, xfer->chip_select,
					command1);

	tegra_spi_writel(tspi, command1, SPI_COMMAND_0);
	tspi->cmd1_reg_val = command1;

	dbgprintf("SPI: SPI_COMMAND_0 def 0x%x and written 0x%lx \
		direction=%x\r\n", (unsigned int)tspi->def_cmd1_reg_val,
		command1, (unsigned int)tspi->cur_direction);

	/* Make sure that Rx and Tx FIFO are empty */
	ret = check_and_clear_fifo(tspi);
	if (ret != 0)
		return ret;

	if (total_fifo_words > SPI_FIFO_DEPTH)
		ret = tegra_spi_start_dma_based_transfer(tspi, xfer);
	else
		ret = tegra_spi_start_cpu_based_transfer(tspi, xfer);

	return ret;
}

int tegra_spi_setup(struct tegra_spi_id *id,
			struct tegra_spi_client_setup *setting)
{
	/* struct tegra_spi_id assumed to be first field in tegra_spi_ctlr */
	struct tegra_spi_ctlr *tspi = (struct tegra_spi_ctlr *)id;
	uint32_t command2_reg;
	int ret;
	struct tegra_spi_client_setup *cdata;

	dbgprintf("SPI: tegra_spi_setup\r\n");
	cdata = &tspi->cdata[setting->chip_select];
	cdata->chip_select = setting->chip_select;
	cdata->cs_setup_clk_count = setting->cs_setup_clk_count;
	cdata->cs_hold_clk_count = setting->cs_hold_clk_count;
	cdata->cs_inactive_cycles = setting->cs_inactive_cycles;
	cdata->set_rx_tap_delay = setting->set_rx_tap_delay;
	cdata->spi_max_clk_rate = setting->spi_max_clk_rate;
	cdata->spi_no_dma = setting->spi_no_dma;

	ret = tegra_spi_clk_set_rate(tspi, setting->spi_max_clk_rate);
	if (ret < 0)
		return ret;
	tspi->spi_clk_rate = setting->spi_max_clk_rate;
	tspi->def_cmd2_reg_val = tegra_spi_readl(tspi, SPI_COMMAND2_0);
	command2_reg = tspi->def_cmd2_reg_val;

	if (cdata->set_rx_tap_delay) {
		if (tspi->spi_clk_rate > SPI_SPEED_TAP_DELAY_MARGIN)
			command2_reg = NV_FLD_SET_DRF_NUM(SPI, COMMAND2,
					Rx_Clk_TAP_DELAY,
					SPI_DEFAULT_RX_TAP_DELAY,
					command2_reg);
	}
	tegra_spi_writel(tspi, command2_reg, SPI_COMMAND2_0);

	return 0;
}

static int tegra_spi_dma_init(struct tegra_spi_ctlr *tspi)
{
	int ret;

	/* TODO: full duplex transactions require 2 channels. DMA POR is to
	 * have only channel for SPI.
	 * To test full duplex, it uses the 3rd channel of I2C.
	 * How useful is this 3rd channel for I2C and how often does Touch use
	 * DMA ? Need further discussion on this.
	 */
	ret = tegra_gpcdma_channel_init(tspi->dma_id, tspi->dma_channel.tx);
	if (ret) {
		error_hook("SPI: DMA channel init failed");
		return -1;
	}

	if (tspi->dma_channel.rx != tspi->dma_channel.tx) {
		ret = tegra_gpcdma_channel_init(tspi->dma_id,
						tspi->dma_channel.rx);
		if (ret) {
			error_hook("SPI: DMA channel init failed");
			tegra_gpcdma_channel_deinit(tspi->dma_id,
							tspi->dma_channel.tx);
			return -1;
		}
	}

	tspi->irq_queue = xQueueCreate(1, sizeof(uint32_t));
	if (!tspi->irq_queue) {
		error_hook("SPI: IRQ queue create failed");
		tegra_gpcdma_channel_deinit(tspi->dma_id,
						tspi->dma_channel.tx);
		if (tspi->dma_channel.rx != tspi->dma_channel.tx)
			tegra_gpcdma_channel_deinit(tspi->dma_id,
							tspi->dma_channel.rx);
		return -1;
	}

	tspi->dma_queue = xQueueCreate(1, sizeof(uint32_t));
	if (!tspi->dma_queue) {
		error_hook("SPI: DMA queue create failed");
		vQueueDelete(tspi->irq_queue);
		tegra_gpcdma_channel_deinit(tspi->dma_id,
						tspi->dma_channel.tx);
		if (tspi->dma_channel.rx != tspi->dma_channel.tx)
			tegra_gpcdma_channel_deinit(tspi->dma_id,
							tspi->dma_channel.rx);
		return -1;
	}

	dbgprintf("DMA initialized with channels tx=%d, rx=%d\n",
			tspi->dma_channel.tx,
			tspi->dma_channel.rx);

	return 0;
}

int tegra_spi_init(struct tegra_spi_id *id,
			struct tegra_spi_master_init *setting)
{
	/* struct tegra_spi_id assumed to be first field in tegra_spi_ctlr */
	struct tegra_spi_ctlr *tspi = (struct tegra_spi_ctlr *)id;
	int ret;

	dbgprintf("SPI: tegra_spi_init\r\n");
	tspi->dma_id = setting->dma_id;
	tspi->dma_channel.tx = setting->dma_channel.tx;
	tspi->dma_channel.rx = setting->dma_channel.rx;
	tspi->spi_clk_rate = setting->spi_max_clk_rate;
	tspi->dma_slave_req = setting->dma_slave_req;
	tspi->busy = false;

#ifndef _NV_BUILD_FPGA_
	ret = tegra_clk_set_rate(tspi->conf.div_clk, tspi->spi_clk_rate);
	if (ret) {
		error_hook("SPI: Failed to set clk freq");
		return -1;
	}

	ret = tegra_clk_enable(tspi->conf.div_clk);
	if (ret) {
		error_hook("SPI: clock enable failed");
		return -1;
	}

	ret = tegra_clk_reset_pulse(tspi->conf.reset, 2);
	if (ret) {
		error_hook("SPI: reset failed");
		tegra_clk_disable(tspi->conf.div_clk);
		return -1;
	}
#endif

	ret = tegra_spi_dma_init(tspi);
	if (ret < 0) {
		error_hook("SPI: dma init failed");
		tegra_clk_disable(tspi->conf.div_clk);
		return ret;
	}

	irq_set_handler(tspi->conf.irq, tegra_spi_irq, id);

	tspi->def_cmd1_reg_val = NV_DRF_DEF(SPI, COMMAND, M_S, MASTER) |
				 NV_DRF_DEF(SPI, COMMAND, En_LE_Byte, FIRST) |
				 NV_DRF_DEF(SPI, COMMAND, CS_SEL, CS0);
	tegra_spi_writel(tspi, tspi->def_cmd1_reg_val, SPI_COMMAND_0);

	return 0;
}

static void tegra_spi_dump_regs(struct tegra_spi_ctlr *tspi)
{
	uint32_t cmd1_reg_val;
	uint32_t fifo_status_reg;
	uint32_t dma_ctrl_reg;
	uint32_t trans_status_reg;

	cmd1_reg_val = tegra_spi_readl(tspi, SPI_COMMAND_0);
	fifo_status_reg = tegra_spi_readl(tspi, SPI_FIFO_STATUS_0);
	dma_ctrl_reg = tegra_spi_readl(tspi, SPI_DMA_CTL_0);
	trans_status_reg = tegra_spi_readl(tspi, SPI_TRANSFER_STATUS_0);

	if (in_interrupt()) {
		dbgprintf_isr("SPI: error CMD_0: 0x%lx, FIFO_STS: 0x%lx\r\n",
			cmd1_reg_val, fifo_status_reg);
		dbgprintf_isr("SPI: error DMA_CTL: 0x%lx, TRANS_STS: 0x%lx\r\n",
			dma_ctrl_reg, trans_status_reg);
	} else {
		dbgprintf("SPI: error CMD_0: 0x%lx, FIFO_STS: 0x%lx\r\n",
			cmd1_reg_val, fifo_status_reg);
		dbgprintf("SPI: error DMA_CTL: 0x%lx, TRANS_STS: 0x%lx\r\n",
			dma_ctrl_reg, trans_status_reg);
	}
}

static int tegra_spi_wait_on_message_xfer(struct tegra_spi_ctlr *tspi)
{
	uint32_t int_source;
	int ret;

	dbgprintf("inside %s\r\n", __func__);
	/* Wait for IRQ notification */
	dbgprintf("SPI: enabling IRQ for transfer\r\n");
	irq_enable(tspi->conf.irq);
	ret = xQueueReceive(tspi->irq_queue, &int_source, SPI_TRANSFER_TIMEOUT);
	irq_disable(tspi->conf.irq);

	if (!ret) {
		error_hook("SPI: transfer timeout");
		tegra_spi_dump_regs(tspi);
		tegra_clk_reset_pulse(tspi->conf.reset, 2);
		if (tspi->is_curr_dma_xfer == true) {
			if (tspi->cur_direction & DATA_DIR_TX)
				tegra_gpcdma_abort(tspi->dma_id,
							tspi->dma_channel.tx);
			if (tspi->cur_direction & DATA_DIR_RX)
				tegra_gpcdma_abort(tspi->dma_id,
							tspi->dma_channel.rx);
		}
		return -1;
	}

	if (tspi->tx_status || tspi->rx_status) {
		error_hook("SPI: Error in transfer");
		if (tspi->is_curr_dma_xfer == true) {
			if (tspi->cur_direction & DATA_DIR_TX)
				tegra_gpcdma_abort(tspi->dma_id,
							tspi->dma_channel.tx);
			if (tspi->cur_direction & DATA_DIR_RX)
				tegra_gpcdma_abort(tspi->dma_id,
							tspi->dma_channel.rx);
		}
		return -1;
	}

	return 0;
}

static int tegra_spi_transfer_remain_message(struct tegra_spi_ctlr *tspi,
	struct tegra_spi_xfer *xfer)
{
	uint32_t total_fifo_words;
	int ret = 0;
	dbgprintf("inside %s\r\n", __func__);

	if (tspi->is_curr_dma_xfer) {
		total_fifo_words = tegra_spi_calculate_curr_xfer_param(tspi,
								xfer);

		/* Make sure that Rx and Tx FIFO are empty */
		ret = check_and_clear_fifo(tspi);
		if (ret != 0)
			return ret;

		if (total_fifo_words > SPI_FIFO_DEPTH)
			ret = tegra_spi_start_dma_based_transfer(tspi, xfer);
		else
			ret = tegra_spi_start_cpu_based_transfer(tspi, xfer);
	} else {
		tegra_spi_calculate_curr_xfer_param(tspi, xfer);
		tegra_spi_start_cpu_based_transfer(tspi, xfer);
	}

	ret = tegra_spi_wait_on_message_xfer(tspi);

	return ret;
}

static int tegra_spi_handle_message(struct tegra_spi_ctlr *tspi,
					struct tegra_spi_xfer *xfer)
{
	uint8_t dma_status;

	dbgprintf("tegra_spi_handle_message\r\n");
	if (!tspi->is_curr_dma_xfer) {
		if (tspi->cur_direction & DATA_DIR_RX) {
			dbgprintf("SPI: filling rx buf\r\n");
			tegra_spi_read_rx_fifo_to_client_rxbuf(tspi, xfer);
			tspi->cur_pos = tspi->cur_rx_pos;
		} else {
			tspi->cur_pos = tspi->cur_tx_pos;
		}
	} else {
		if (!xQueueReceive(tspi->dma_queue, &dma_status,
						SPI_TRANSFER_TIMEOUT)) {
			error_hook("SPI: DMA transfer timeout");
			tegra_spi_dump_regs(tspi);
			tegra_clk_reset_pulse(tspi->conf.reset, 2);
			if (tspi->cur_direction & DATA_DIR_TX)
				tegra_gpcdma_abort(tspi->dma_id,
						tspi->dma_channel.tx);
			if (tspi->cur_direction & DATA_DIR_RX)
				tegra_gpcdma_abort(tspi->dma_id,
						tspi->dma_channel.rx);
			return -1;
		}

		if (tspi->cur_direction & DATA_DIR_TX) {
			tspi->cur_tx_pos += tspi->curr_dma_words * tspi->bytes_per_word;
			tspi->cur_pos = tspi->cur_tx_pos;
		}
		if (tspi->cur_direction & DATA_DIR_RX) {
			tegra_spi_copy_spi_rxbuf_to_client_rxbuf(tspi, xfer);
			tspi->cur_pos = tspi->cur_rx_pos;
		}
	}

	return 0;
}

int tegra_spi_transfer(struct tegra_spi_id *id, struct tegra_spi_xfer *xfer)
{
	/* struct tegra_spi_id assumed to be first field in tegra_spi_ctlr */
	struct tegra_spi_ctlr *tspi = (struct tegra_spi_ctlr *)id;
	int ret = 0;

	dbgprintf("SPI: tegra_spi_transfer\r\n");
	tspi->busy = true;
	if (xfer->flags & BIT(TEGRA_SPI_XFER_FIRST_MSG))
		ret = tegra_spi_start_transfer_one(tspi, xfer);
	else
		ret = tegra_spi_transfer_remain_message(tspi, xfer);
	if (ret < 0) {
		error_hook("SPI: cannot start transfer");
		goto exit;
	}
	ret = tegra_spi_wait_on_message_xfer(tspi);
	if (ret)
		goto exit;
	ret = tegra_spi_handle_message(tspi, xfer);
	if (ret)
		goto exit;
	if (tspi->cur_pos == xfer->len)
		goto exit;

	while (1) {
		ret = tegra_spi_transfer_remain_message(tspi, xfer);
		if (ret)
			goto exit;
		ret = tegra_spi_handle_message(tspi, xfer);
		if (ret)
			goto exit;
		if (tspi->cur_pos == xfer->len)
			break;
	}

exit:
	dbgprintf("SPI: transfer complete\r\n");
	/* de-assert CS to end the transaction */
	if (xfer->flags & BIT(TEGRA_SPI_XFER_LAST_MSG)) {
		tegra_spi_writel(tspi, tspi->def_cmd1_reg_val, SPI_COMMAND_0);
		dbgprintf("SPI: message complete\r\n");
	}
	tspi->busy = false;

	return ret;
}

static void handle_cpu_based_err_xfer(struct tegra_spi_ctlr *tspi)
{
	dbgprintf_isr("inside %s\r\n", __func__);
	if (tspi->tx_status || tspi->rx_status)
		tegra_clk_reset_pulse(tspi->conf.reset, 2);
}

static void handle_dma_based_err_xfer(struct tegra_spi_ctlr *tspi)
{
	int err = 0;

	dbgprintf_isr("inside %s\r\n", __func__);
	/* Abort DMA if any error */
	if ((tspi->cur_direction & DATA_DIR_TX) && (tspi->tx_status))
		err += 1;

	if ((tspi->cur_direction & DATA_DIR_RX) && (tspi->rx_status))
		err += 2;

	if (err)
		tegra_clk_reset_pulse(tspi->conf.reset, 2);
}

void tegra_spi_irq(void *data)
{
	struct tegra_spi_id *id = data;
	/* struct tegra_spi_id assumed to be first field in tegra_spi_ctlr */
	struct tegra_spi_ctlr *tspi = (struct tegra_spi_ctlr *)id;

	uint32_t cmd1;
	BaseType_t higher_prio_task_woken = pdFALSE;

	dbgprintf_isr("inside %s\r\n", __func__);
	cmd1 = tegra_spi_readl(tspi, SPI_COMMAND_0);
	tspi->status_reg = tegra_spi_readl(tspi, SPI_FIFO_STATUS_0);
	if (cmd1 & NV_DRF_DEF(SPI, COMMAND, Tx_EN, ENABLE))
		tspi->tx_status = tspi->status_reg &
					(NV_DRF_DEF(SPI, FIFO_STATUS,
					TX_FIFO_UNF, ERROR) |
					 NV_DRF_DEF(SPI, FIFO_STATUS,
					TX_FIFO_OVF, ERROR));

	if (cmd1 & NV_DRF_DEF(SPI, COMMAND, Rx_EN, ENABLE))
		tspi->rx_status = tspi->status_reg &
					(NV_DRF_DEF(SPI, FIFO_STATUS,
					RX_FIFO_UNF, ERROR) |
					NV_DRF_DEF(SPI, FIFO_STATUS,
					RX_FIFO_OVF, ERROR));

	if (tspi->tx_status || tspi->rx_status)
		tegra_spi_dump_regs(tspi);
	tegra_spi_clear_status(tspi);
	if (!(cmd1 & NV_DRF_DEF(SPI, COMMAND, Tx_EN, ENABLE)) &&
		!(cmd1 & NV_DRF_DEF(SPI, COMMAND, Rx_EN, ENABLE)))
		error_hook("SPI: spurious interrupt");

	if (!tspi->is_curr_dma_xfer)
		handle_cpu_based_err_xfer(tspi);
	else
		handle_dma_based_err_xfer(tspi);

	if (!xQueueSendToBackFromISR(tspi->irq_queue, &tspi->status_reg,
					&higher_prio_task_woken))
		error_hook("xQueueSendFromISR(spi_irq) failed");

	portYIELD_FROM_ISR(higher_prio_task_woken);
}
