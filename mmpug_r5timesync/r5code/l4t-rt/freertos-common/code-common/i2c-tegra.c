/*
 * Copyright (c) 2014-2016 NVIDIA CORPORATION.  All rights reserved.
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
#include <inttypes.h>

#include <FreeRTOS.h>
#include <task.h>

#include <nvrm_drf.h>

#include <delay.h>
#include <i2c-tegra-priv.h>
#include <i2c-fifo-regconf.h>
#include <irqs.h>
#include <macros.h>
#include <printf-isr.h>
#include <reg-access.h>

#if 0
#define dbgprintf printf
#define dbgprintf_isr printf_isr
#else
static inline void dbgprintf(const char *fmt, ...)
{
}
static inline void dbgprintf_isr(const char *fmt, ...)
{
}
#endif

#define errprintf printf

#define I2C_CNFG_0					0x000
#define I2C_CNFG_0_DEBOUNCE_CNT_RANGE			14:12
#define I2C_CNFG_0_NEW_MASTER_FSM			BIT(11)
#define I2C_CNFG_0_PACKET_MODE_EN			BIT(10)

#define I2C_SL_CNFG_0					0x020
#define I2C_SL_CNFG_0_NACK				BIT(1)
#define I2C_SL_CNFG_0_NEWSL				BIT(2)

#define I2C_TX_PACKET_FIFO_0				0x050

#define I2C_RX_FIFO_0					0x054

#define I2C_STATUS_0					0x01c
#define I2C_STATUS_0_BUSY				BIT(8)

#define I2C_INT_MASK_0					0x064
#define I2C_INT_MASK_0_PACKET_XFER_COMPLETE		BIT(7)
#define I2C_INT_MASK_0_ALL_PACKETS_XFER_COMPLETE	BIT(6)
#define I2C_INT_MASK_0_NOACK				BIT(3)
#define I2C_INT_MASK_0_ARB_LOST				BIT(2)
#define I2C_INT_MASK_0_TFIFO_DATA_REQ			BIT(1)
#define I2C_INT_MASK_0_RFIFO_DATA_REQ			BIT(0)

#define I2C_INT_STATUS_0				0x068
#define I2C_INT_STATUS_0_PACKET_XFER_COMPLETE		BIT(7)
#define I2C_INT_STATUS_0_ALL_PACKETS_XFER_COMPLETE	BIT(6)
#define I2C_INT_STATUS_0_NOACK				BIT(3)
#define I2C_INT_STATUS_0_ARB_LOST			BIT(2)
#define I2C_INT_STATUS_0_TFIFO_DATA_REQ			BIT(1)
#define I2C_INT_STATUS_0_RFIFO_DATA_REQ			BIT(0)

#define I2C_CLK_DIVISOR_0				0x06c
#define I2C_CLK_DIVISOR_0_STD_FAST_MODE_RANGE		31:16
#define I2C_CLK_DIVISOR_0_HSMODE_RANGE			15:0

#define I2C_INT_SOURCE_0				0x70
#define I2C_INT_SOURCE_0_PACKET_XFER_COMPLETE		BIT(7)
#define I2C_INT_SOURCE_0_ALL_PACKETS_XFER_COMPLETE	BIT(6)
#define I2C_INT_SOURCE_0_NOACK				BIT(3)
#define I2C_INT_SOURCE_0_ARB_LOST			BIT(2)
#define I2C_INT_SOURCE_0_TFIFO_DATA_REQ			BIT(1)
#define I2C_INT_SOURCE_0_RFIFO_DATA_REQ			BIT(0)

#define I2C_CONFIG_LOAD_0				0x08c
#define I2C_CONFIG_LOAD_0_TIMEOUT_CONFIG_LOAD		BIT(2)
#define I2C_CONFIG_LOAD_0_SLV_CONFIG_LOAD		BIT(1)
#define I2C_CONFIG_LOAD_0_MSTR_CONFIG_LOAD		BIT(0)

#define PACKET_HEADER0_0_HEADER_SIZE_RANGE		29:28
#define PACKET_HEADER0_0_PACKET_ID_RANGE		23:16
#define PACKET_HEADER0_0_CONT_ID_RANGE			15:12
#define PACKET_HEADER0_0_PROTOCOL_RANGE			7:4
#define PACKET_HEADER0_0_PROTOCOL_I2C			1
#define PACKET_HEADER0_0_PKT_TYPE_RANGE			2:0
#define PACKET_HEADER0_0_PKT_TYPE_REQUEST		0

#define PACKET_HEADER1_0_PAYLOAD_SIZE_RANGE		11:0

#define PACKET_HEADER_I2C_0_HIGHSPEED_MODE		BIT(22)
#define PACKET_HEADER_I2C_0_CONT_ON_NAK			BIT(21)
#define PACKET_HEADER_I2C_0_SEND_START_BYTE		BIT(20)
#define PACKET_HEADER_I2C_0_READ			BIT(19)
#define PACKET_HEADER_I2C_0_10BIT_ADDR			BIT(18)
#define PACKET_HEADER_I2C_0_IE_ENABLE			BIT(17)
#define PACKET_HEADER_I2C_0_REPEAT_START		BIT(16)
#define PACKET_HEADER_I2C_0_CONTINUE_XFER		BIT(15)
#define PACKET_HEADER_I2C_0_ADDR_RANGE			14:12
#define PACKET_HEADER_I2C_0_SLAVE_ADDR_RANGE		9:1

static inline uint32_t tegra_i2c_readl(struct tegra_i2c_ctlr *ctlr, uint32_t offset)
{
	offset *= ctlr->conf.offset_mult;
	offset += ctlr->conf.base_addr;
	return readl(offset);
}

static void tegra_i2c_writel(struct tegra_i2c_ctlr *ctlr, uint32_t val, uint32_t offset)
{
	offset *= ctlr->conf.offset_mult;
	offset += ctlr->conf.base_addr;
	writel(val, offset);
}

static void tegra_i2c_select_tx_xfer(struct tegra_i2c_ctlr *ctlr, uint32_t xfer_idx)
{
	dbgprintf("I2C: select TX xfer id %" PRIu32 " @%p\r\n", xfer_idx, (void *)&ctlr->xfers[xfer_idx]);

	ctlr->cur_tx_xfer_idx = xfer_idx;
	ctlr->cur_tx_xfer_sent_header = false;
	ctlr->cur_tx_xfer_buf = ctlr->xfers[xfer_idx].buf;
	ctlr->cur_tx_xfer_count_remaining = ctlr->xfers[xfer_idx].count;
}

static void tegra_i2c_select_next_tx_xfer(struct tegra_i2c_ctlr *ctlr)
{
	uint32_t val;

	ctlr->cur_tx_xfer_idx++;

	if (ctlr->cur_tx_xfer_idx < ctlr->num_xfers) {
		tegra_i2c_select_tx_xfer(ctlr, ctlr->cur_tx_xfer_idx);
		return;
	}

	dbgprintf("I2C: sent all TX xfers\r\n");
	val = tegra_i2c_readl(ctlr, I2C_INT_MASK_0);
	val &= ~I2C_INT_SOURCE_0_TFIFO_DATA_REQ;
	tegra_i2c_writel(ctlr, val, I2C_INT_MASK_0);
}

static void tegra_i2c_select_first_tx_xfer(struct tegra_i2c_ctlr *ctlr)
{
	tegra_i2c_select_tx_xfer(ctlr, 0);
}

static void tegra_i2c_select_rx_xfer(struct tegra_i2c_ctlr *ctlr, uint32_t xfer_idx)
{
	dbgprintf("I2C: select RX xfer id %" PRIu32 " @%p\r\n", xfer_idx, (void *)&ctlr->xfers[xfer_idx]);

	ctlr->cur_rx_xfer_idx = xfer_idx;
	ctlr->cur_rx_xfer_buf = ctlr->xfers[xfer_idx].buf;
	ctlr->cur_rx_xfer_count_remaining = ctlr->xfers[xfer_idx].count;
}

static void tegra_i2c_select_next_rx_xfer(struct tegra_i2c_ctlr *ctlr)
{
	uint32_t val;

	for (;;) {
		ctlr->cur_rx_xfer_idx++;
		if (ctlr->cur_rx_xfer_idx >= ctlr->num_xfers)
			break;

		if (!ctlr->xfers[ctlr->cur_rx_xfer_idx].is_read)
			continue;

		tegra_i2c_select_rx_xfer(ctlr, ctlr->cur_rx_xfer_idx);
		return;
	}

	dbgprintf("I2C: drained all RX xfers\r\n");
	val = tegra_i2c_readl(ctlr, I2C_INT_MASK_0);
	val &= ~I2C_INT_SOURCE_0_RFIFO_DATA_REQ;
	tegra_i2c_writel(ctlr, val, I2C_INT_MASK_0);
}

static void tegra_i2c_select_first_rx_xfer(struct tegra_i2c_ctlr *ctlr)
{
	ctlr->cur_rx_xfer_idx = -1;
	tegra_i2c_select_next_rx_xfer(ctlr);
}

static uint32_t tegra_i2c_txfifo_entries_avail(struct tegra_i2c_ctlr *ctlr)
{
	uint32_t fifo_status, fifo_free_entries;

	fifo_status = tegra_i2c_readl(ctlr, I2C_FIFO_STATUS_0);
	fifo_free_entries = NV_DRF_VAL(I2C, FIFO_STATUS, TX_FIFO_EMPTY_CNT, fifo_status);

	dbgprintf("I2C: %" PRIu32 " TXFIFO entries free\r\n", fifo_free_entries);

	return fifo_free_entries;
}

static uint32_t tegra_i2c_rxfifo_entries_filled(struct tegra_i2c_ctlr *ctlr)
{
	uint32_t fifo_status, fifo_filled_entries;

	fifo_status = tegra_i2c_readl(ctlr, I2C_FIFO_STATUS_0);
	fifo_filled_entries = NV_DRF_VAL(I2C, FIFO_STATUS, RX_FIFO_FULL_CNT, fifo_status);

	dbgprintf("I2C: %" PRIu32 " RXFIFO entries filled\r\n", fifo_filled_entries);

	return fifo_filled_entries;
}

static void tegra_i2c_fill_txfifo_header(struct tegra_i2c_ctlr *ctlr)
{
	struct tegra_i2c_xfer *xfer = &(ctlr->xfers[ctlr->cur_tx_xfer_idx]);
	bool is_last = ctlr->cur_tx_xfer_idx == (ctlr->num_xfers - 1);
	uint32_t val;

	if (tegra_i2c_txfifo_entries_avail(ctlr) < 3)
		return;

	dbgprintf("I2C: fill TX: Writing header for %p\r\n", (void *)xfer);

	dbgprintf("I2C: fill TX: Transfer dev 0x%02" PRIx32 " size %" PRIu32 " %s%s\r\n",
		(uint32_t)xfer->i2c_addr, xfer->count,
		xfer->is_read ? "read" : "write",
		is_last ? " last" : "");

	val = NV_DRF_NUM(PACKET, HEADER0, HEADER_SIZE, 1 - 1) |
		NV_DRF_NUM(PACKET, HEADER0, PACKET_ID, 1) |
		NV_DRF_NUM(PACKET, HEADER0, CONT_ID, 0) |
		NV_DRF_DEF(PACKET, HEADER0, PROTOCOL, I2C) |
		NV_DRF_DEF(PACKET, HEADER0, PKT_TYPE, REQUEST);
	tegra_i2c_writel(ctlr, val, I2C_TX_PACKET_FIFO_0);

	val = NV_DRF_NUM(PACKET, HEADER1, PAYLOAD_SIZE, xfer->count - 1);
	tegra_i2c_writel(ctlr, val, I2C_TX_PACKET_FIFO_0);

	val = NV_DRF_NUM(PACKET, HEADER_I2C, SLAVE_ADDR, xfer->i2c_addr);
	if (!is_last)
		val |= PACKET_HEADER_I2C_0_REPEAT_START;
	if (xfer->is_read)
		val |= PACKET_HEADER_I2C_0_READ;
	tegra_i2c_writel(ctlr, val, I2C_TX_PACKET_FIFO_0);

	ctlr->cur_tx_xfer_sent_header = true;
}

static void tegra_i2c_fill_txfifo_txdata(struct tegra_i2c_ctlr *ctlr)
{
	uint32_t space_in_fifo;
	uint32_t bytes_to_write;
	uint32_t words_to_write;
	uint32_t val;

	space_in_fifo = tegra_i2c_txfifo_entries_avail(ctlr) * 4;
	if (space_in_fifo < ctlr->cur_tx_xfer_count_remaining)
		bytes_to_write = space_in_fifo;
	else
		bytes_to_write = ctlr->cur_tx_xfer_count_remaining;
	ctlr->cur_tx_xfer_count_remaining -= bytes_to_write;

	dbgprintf("I2C: fill TX: %" PRIu32 " bytes to write\r\n", bytes_to_write);

	words_to_write = bytes_to_write / 4;
	if (words_to_write)
		dbgprintf("I2C: fill TX: %" PRIu32 " words to write\r\n", words_to_write);
	while (words_to_write--) {
		memcpy(&val, ctlr->cur_tx_xfer_buf, 4);
		ctlr->cur_tx_xfer_buf += 4;
		tegra_i2c_writel(ctlr, val, I2C_TX_PACKET_FIFO_0);
	}

	bytes_to_write &= 3;
	if (bytes_to_write) {
		dbgprintf("I2C: fill TX: %" PRIu32 " bytes remainder to write\r\n", bytes_to_write);
		memcpy(&val, ctlr->cur_tx_xfer_buf, bytes_to_write);
		tegra_i2c_writel(ctlr, val, I2C_TX_PACKET_FIFO_0);
	}
}

static void tegra_i2c_fill_txfifo(struct tegra_i2c_ctlr *ctlr)
{
	for (;;) {
		if (!ctlr->cur_tx_xfer_sent_header) {
			dbgprintf("I2c: fill TX: attempting current header\r\n");
			tegra_i2c_fill_txfifo_header(ctlr);
			if (!ctlr->cur_tx_xfer_sent_header)
				return;
			dbgprintf("I2c: fill TX: current header sent\r\n");
		}
		if (!ctlr->xfers[ctlr->cur_tx_xfer_idx].is_read) {
			dbgprintf("I2c: fill TX: attempting current data buffer\r\n");
			tegra_i2c_fill_txfifo_txdata(ctlr);
			if (ctlr->cur_tx_xfer_count_remaining)
				return;
			dbgprintf("I2c: fill TX: current data buffer sent\r\n");
		}
		tegra_i2c_select_next_tx_xfer(ctlr);
		if (ctlr->cur_tx_xfer_idx >= ctlr->num_xfers)
			return;
	}
}

static void tegra_i2c_drain_rxfifo_rxdata(struct tegra_i2c_ctlr *ctlr)
{
	uint32_t filled_entries_in_fifo;
	uint32_t bytes_to_read;
	uint32_t words_to_read;
	uint32_t val;

	filled_entries_in_fifo = tegra_i2c_rxfifo_entries_filled(ctlr);
	if ((filled_entries_in_fifo * 4) < ctlr->cur_rx_xfer_count_remaining)
		bytes_to_read = (filled_entries_in_fifo * 4);
	else
		bytes_to_read = ctlr->cur_rx_xfer_count_remaining;
	ctlr->cur_rx_xfer_count_remaining -= bytes_to_read;

	dbgprintf("I2C: drain RX: %" PRIu32 " bytes to read\r\n", bytes_to_read);

	words_to_read = bytes_to_read / 4;
	while (words_to_read--) {
		val = tegra_i2c_readl(ctlr, I2C_RX_FIFO_0);
		memcpy(ctlr->cur_rx_xfer_buf, &val, 4);
		ctlr->cur_rx_xfer_buf += 4;
	}

	bytes_to_read &= 3;
	if (bytes_to_read) {
		val = tegra_i2c_readl(ctlr, I2C_RX_FIFO_0);
		memcpy(ctlr->cur_rx_xfer_buf, &val, bytes_to_read);
		ctlr->cur_rx_xfer_buf += bytes_to_read;
	}
}

static void tegra_i2c_drain_rxfifo(struct tegra_i2c_ctlr *ctlr)
{
	for (;;) {
		dbgprintf("I2c: drain RX: attempting current data buffer\r\n");
		tegra_i2c_drain_rxfifo_rxdata(ctlr);
		if (ctlr->cur_rx_xfer_count_remaining)
			return;
		dbgprintf("I2c: drain RX: current buffer data complete\r\n");
		tegra_i2c_select_next_rx_xfer(ctlr);
		if (ctlr->cur_rx_xfer_idx >= ctlr->num_xfers)
			return;
	}
}

void tegra_i2c_irq(void *data)
{
	struct tegra_i2c_id *id = data;
	/* struct tegra_i2c_id assumed to be first field in tegra_i2c_ctrl */
	struct tegra_i2c_ctlr *ctlr = (struct tegra_i2c_ctlr *)id;
	uint32_t val;
	BaseType_t higher_prio_task_woken = pdFALSE;

	val = tegra_i2c_readl(ctlr, I2C_INT_SOURCE_0);
	tegra_i2c_writel(ctlr, val, I2C_INT_STATUS_0);

	dbgprintf_isr("I2C: ISR: I2C_INT_SOURCE_0 = 0x%" PRIx32 "\r\n", val);

	if (val & I2C_INT_SOURCE_0_TFIFO_DATA_REQ)
		tegra_i2c_fill_txfifo(ctlr);
	if (ctlr->cur_rx_xfer_idx < ctlr->num_xfers)
		tegra_i2c_drain_rxfifo(ctlr);
	if (val & (I2C_INT_MASK_0_NOACK | I2C_INT_MASK_0_ARB_LOST | I2C_INT_MASK_0_ALL_PACKETS_XFER_COMPLETE))
		xQueueSendToBackFromISR(ctlr->irq_queue, &val, &higher_prio_task_woken);

	dbgprintf_isr("I2C: ISR complete\r\n");

	portYIELD_FROM_ISR(higher_prio_task_woken);
}

static int tegra_i2c_config_and_flush_fifos(struct tegra_i2c_ctlr *ctlr)
{
	uint32_t tx_trig, rx_trig, val;
	uint64_t tstart;

	/*
	 * Trigger TX interrupts when there's enough space to place an entire
	 * transaction header into the FIFO. This simplifies the logic that
	 * streams transactions into the FIFO. A future optimization might be
	 * to vary this value based on whether we're waiting for space to fill
	 * data or a transaction header.
	 */
	tx_trig = 3;
	/*
	 * Trigger RX interrupts when the RX FIFO is almost full.
	 * This reduces the number of interrupts that are serviced, while
	 * hopefully never blocking the actual bus transfer due to the FIFO
	 * filling up.
	 */
	rx_trig = 8 - 2;

	val = I2C_FIFO_CONTROL_0_TX_FLUSH |
		I2C_FIFO_CONTROL_0_RX_FLUSH |
		NV_DRF_NUM(I2C, FIFO_CONTROL, TX_TRIG, tx_trig) |
		NV_DRF_NUM(I2C, FIFO_CONTROL, RX_TRIG, rx_trig);
	tegra_i2c_writel(ctlr, val, I2C_FIFO_CONTROL_0);
	tstart = get_time_ticks();
	for (;;) {
		val = tegra_i2c_readl(ctlr, I2C_FIFO_CONTROL_0);
		val &= I2C_FIFO_CONTROL_0_TX_FLUSH | I2C_FIFO_CONTROL_0_RX_FLUSH;
		if (!val)
			return 0;
		if (get_time_delta_us(tstart) > 100)
			return 1;
		taskYIELD();
	}
}

static int tegra_i2c_config_load(struct tegra_i2c_ctlr *ctlr)
{
	uint32_t val;
	uint64_t tstart;

	val = I2C_CONFIG_LOAD_0_SLV_CONFIG_LOAD |
		I2C_CONFIG_LOAD_0_MSTR_CONFIG_LOAD;
	tegra_i2c_writel(ctlr, val, I2C_CONFIG_LOAD_0);
	tstart = get_time_ticks();
	for (;;) {
		val = tegra_i2c_readl(ctlr, I2C_CONFIG_LOAD_0);
		if (!val)
			return 0;
		/*
		 * The following timeout should be as follows:
		 * timeout >= 10 * (1 / lowest_clock_frequency)
		 * TIMEOUT_CONFIG_LOAD can use I2C_SLOW_CLK which could be
		 * 32Khz. So the maximum expected timeout is ~320 usecs.
		 * To account for unexpected conditions, set the timeout
		 * to a higher value.
		 */
		if (get_time_delta_us(tstart) > 640)
			return 1;
		taskYIELD();
	}
}

static int tegra_i2c_init_hw(struct tegra_i2c_ctlr *ctlr)
{
	uint32_t val;
	int ret;

#ifndef _I2C_TEGRA_DISABLE_CAR_ACCESS
	tegra_clk_reset_pulse(ctlr->conf.reset, 2);
#endif

	tegra_i2c_writel(ctlr, 0, I2C_INT_MASK_0);

	val = I2C_CNFG_0_PACKET_MODE_EN |
		I2C_CNFG_0_NEW_MASTER_FSM |
		NV_DRF_NUM(I2C, CNFG, DEBOUNCE_CNT, 2);
	tegra_i2c_writel(ctlr, val, I2C_CNFG_0);

	tegra_i2c_writel(ctlr, ctlr->clk_divisor, I2C_CLK_DIVISOR_0);

	val = I2C_SL_CNFG_0_NEWSL | I2C_SL_CNFG_0_NACK;
	tegra_i2c_writel(ctlr, val, I2C_SL_CNFG_0);

	ret = tegra_i2c_config_and_flush_fifos(ctlr);
	if (ret)
		return ret;
	ret = tegra_i2c_config_load(ctlr);
	if (ret)
		return ret;

	irq_set_handler(ctlr->conf.irq, tegra_i2c_irq, ctlr);

	return 0;
}

static void tegra_i2c_set_clk_rate(struct tegra_i2c_ctlr *ctlr,
	uint32_t bus_clk_rate)
{
	/*
	 * FIXME: This needs to be fleshed out;
	 * find out what the complete algorithm should be.
	 */
#ifdef _NV_BUILD_FPGA_
	/*
	 * The I2C module clock frequency is 10Mhz on unitFPGA.
	 * scl_freq (std/fast/fm+ modes) = ClkSourceFreq / ((tlow + thigh + 2) * (N + 1))
	 * N = (ClkSourceFreq / ((tlow + thigh + 2) * scl_freq)) - 1
	 * To set scl_freq = 100K then
	 * N = (10M / ((2 + 4 + 2) * 100k)) - 1
	 *   = 0xC
	 */
	const uint32_t clk_divisor_std_fast_mode = 0xC;
#else
	const uint32_t clk_divisor_std_fast_mode = 0x19;
#endif
	const uint32_t clk_divisor_hs_mode = 2;

#ifndef _I2C_TEGRA_DISABLE_CAR_ACCESS
#define I2C_CLK_MULTIPLIER_STD_FAST_MODE 8
	uint32_t clk_multiplier = I2C_CLK_MULTIPLIER_STD_FAST_MODE * (clk_divisor_std_fast_mode + 1);

	tegra_clk_set_rate(ctlr->conf.slow_clk, 32768);
	tegra_clk_set_rate(ctlr->conf.div_clk, bus_clk_rate * clk_multiplier);
#endif

	ctlr->clk_divisor =
		NV_DRF_NUM(I2C, CLK_DIVISOR, HSMODE, clk_divisor_hs_mode) |
		NV_DRF_NUM(I2C, CLK_DIVISOR, STD_FAST_MODE, clk_divisor_std_fast_mode);
}

int tegra_i2c_init(struct tegra_i2c_id *id, uint32_t bus_clk_rate)
{
	int ret;
	/* struct tegra_i2c_id assumed to be first field in tegra_i2c_ctrl */
	struct tegra_i2c_ctlr *ctlr = (struct tegra_i2c_ctlr *)id;

	ctlr->irq_queue = xQueueCreate(1, sizeof(uint32_t));
	if (!ctlr->irq_queue)
		return 1;

	tegra_i2c_set_clk_rate(ctlr, bus_clk_rate);
#ifndef _I2C_TEGRA_DISABLE_CAR_ACCESS
	tegra_clk_enable(ctlr->conf.slow_clk);
	tegra_clk_enable(ctlr->conf.div_clk);
#endif

	ret = tegra_i2c_init_hw(ctlr);
	if (ret)
		return ret;

	return 0;
}

void tegra_i2c_fini(struct tegra_i2c_id *id)
{
	/* struct tegra_i2c_id assumed to be first field in tegra_i2c_ctrl */
	struct tegra_i2c_ctlr *ctlr = (struct tegra_i2c_ctlr *)id;

	vQueueDelete(ctlr->irq_queue);
}

uint32_t tegra_i2c_phys_addr(struct tegra_i2c_id *id)
{
	struct tegra_i2c_ctlr *ctlr = (struct tegra_i2c_ctlr *)id;

	return ctlr->conf.base_addr;
}

int tegra_i2c_transfer(struct tegra_i2c_id *id, struct tegra_i2c_xfer *xfers, uint32_t num_xfers)
{
	/* struct tegra_i2c_id assumed to be first field in tegra_i2c_ctrl */
	struct tegra_i2c_ctlr *ctlr = (struct tegra_i2c_ctlr *)id;
	uint32_t val;
	uint32_t int_source;
	int ret;

	if (ctlr->broken)
		return 1;

	if (!num_xfers)
		return 0;

	dbgprintf("I2C: %" PRIu32 " xfers @%p\r\n", num_xfers, (void *)xfers);

	ctlr->xfers = xfers;
	ctlr->num_xfers = num_xfers;
	tegra_i2c_select_first_tx_xfer(ctlr);
	tegra_i2c_select_first_rx_xfer(ctlr);

	/* Drain any old events, clear old IRQs */
	xQueueReceive(ctlr->irq_queue, &int_source, 0);
	tegra_i2c_writel(ctlr, 0xffffffff, I2C_INT_STATUS_0);

	/* Enable all termination IRQs, and all FIFO fill/drain IRQs */
	val = I2C_INT_MASK_0_NOACK |
		I2C_INT_MASK_0_ARB_LOST |
		I2C_INT_MASK_0_ALL_PACKETS_XFER_COMPLETE |
		I2C_INT_MASK_0_RFIFO_DATA_REQ |
		I2C_INT_MASK_0_TFIFO_DATA_REQ;
	tegra_i2c_writel(ctlr, val, I2C_INT_MASK_0);

	/* May adjust I2C_INT_MASK_0 to disable TX/RX FIFO IRQs if needed */
	tegra_i2c_fill_txfifo(ctlr);

	/* Wait for IRQ notification */
	dbgprintf("I2C: enabling IRQ for transfer\r\n");
	irq_enable(ctlr->conf.irq);
	if (!xQueueReceive(ctlr->irq_queue, &int_source, 100 /* ms */ / portTICK_PERIOD_MS))
		int_source = 0;
	irq_disable(ctlr->conf.irq);
	ret = 0;
	if (int_source & (I2C_INT_SOURCE_0_NOACK | I2C_INT_SOURCE_0_ARB_LOST)) {
		errprintf("I2C: Xfer: error IRQ status: 0x%" PRIx32 "\r\n", int_source);
		ret = 1;
	} else if (!(int_source & I2C_INT_SOURCE_0_ALL_PACKETS_XFER_COMPLETE)) {
		errprintf("I2C: Xfer: timeout\r\n");
		ret = 1;
	}
	if (ctlr->cur_tx_xfer_idx != num_xfers) {
		errprintf("I2C: Xfer: didn't TX all packets (cur %" PRIu32 " num %" PRIu32 ")\r\n", ctlr->cur_tx_xfer_idx, num_xfers);
		ret = 1;
	}
	if (ctlr->cur_rx_xfer_idx != num_xfers) {
		errprintf("I2C: Xfer: didn't RX all packets (cur %" PRIu32 " num %" PRIu32 ")\r\n", ctlr->cur_rx_xfer_idx,  num_xfers);
		errprintf("I2C: remaining RX bytes: %" PRIu32 "\r\n", ctlr->cur_rx_xfer_count_remaining);
		ret = 1;
	}
	val = tegra_i2c_txfifo_entries_avail(ctlr);
	if (val != I2C_FIFO_STATUS_0_TX_FIFO_EMPTY_CNT_MAX) {
		errprintf("I2C: Xfer: TX FIFO non-empty (free %" PRIu32 ")\r\n", val);
		ret = 1;
	}
	val = tegra_i2c_rxfifo_entries_filled(ctlr);
	if (val) {
		errprintf("I2C: Xfer: RX FIFO non-empty (filled %" PRIu32 ")\r\n", val);
		ret = 1;
	}

	if (ret) {
		ret = tegra_i2c_init_hw(ctlr);
		if (ret)
			ctlr->broken = true;
		return 1;
	}

	dbgprintf("I2C: Xfer: OK\r\n");

	return 0;
}


int tegra_i2c_suspend(struct tegra_i2c_id *id)
{
	struct tegra_i2c_ctlr *ctlr = (struct tegra_i2c_ctlr*) id;
	uint32_t busy;

	do {
		busy = tegra_i2c_readl(ctlr, I2C_STATUS_0);
		busy &= ~I2C_STATUS_0_BUSY;
	} while (busy);

#ifndef _I2C_TEGRA_DISABLE_CAR_ACCESS
	tegra_clk_reset_set(ctlr->conf.reset);
	tegra_clk_disable(ctlr->conf.div_clk);
	tegra_clk_disable(ctlr->conf.slow_clk);
#endif
	return 0;
}

int tegra_i2c_resume(struct tegra_i2c_id *id)
{
	struct tegra_i2c_ctlr *ctlr = (struct tegra_i2c_ctlr*) id;

#ifndef _I2C_TEGRA_DISABLE_CAR_ACCESS
	tegra_clk_enable(ctlr->conf.slow_clk);
	tegra_clk_enable(ctlr->conf.div_clk);
	tegra_clk_reset_clear(ctlr->conf.reset);
#endif
	return tegra_i2c_init_hw(ctlr);
}
