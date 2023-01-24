/*
 * Copyright (c) 2018 NVIDIA CORPORATION.  All rights reserved.
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

#include <inttypes.h>
#include <errno.h>

#include <macros.h>
#include <tke-tegra.h>
#include <hsp-tegra.h>
#include <compiler.h>

#include <combined-uart-tegra.h>
#include <combined-uart-tegra-priv.h>

/* Shared mailbox protocol bits */
#define COMB_UART_PKT_SIZE_SHIFT    (24U)
#define COMB_UART_PKT_SIZE_MASK     (3U << COMB_UART_PKT_SIZE_SHIFT)
#define COMB_UART_PKT_FLUSH_BIT     (1U << 26U)

/* RX/TX timeout */
#define COMB_UART_TX_TIMEOUT_MS     (500U)
#define COMB_UART_RX_TIMEOUT_MS     (500U)

#define COMB_UART_RX_TIMEOUT_TICKS (COMB_UART_RX_TIMEOUT_MS / portTICK_PERIOD_MS)

#define COMBINED_UART_RX_Q_SIZE     (256)

/**
 * Wait for TX to clear, or until COMB_UART_TX_TIMEOUT_MS timeout.
 *
 * Return 0 on success, -ETIMEDOUT on failure.
 */
static inline int32_t combined_uart_wait_tx_empty(struct tegra_combined_uart_ctlr *ctlr)
{
	int32_t ret = 0;

	if (!tegra_hsp_sm_is_empty(ctlr->conf.tx_hsp_id, ctlr->conf.tx_sm_num)) {
		ret = -ETIMEDOUT;

		uint32_t tsc_hi = 0, tsc_lo = 0;
		tegra_tke_get_tsc(&tsc_hi, &tsc_lo);

		do {
			if (tegra_hsp_sm_is_empty(ctlr->conf.tx_hsp_id, ctlr->conf.tx_sm_num)) {
				ret = 0;
				break;
			}

		} while (tegra_tke_get_elapsed_usec(tsc_hi, tsc_lo) < (COMB_UART_TX_TIMEOUT_MS * 1000U));
	}

	return ret;
}

/**
 * Helper function to write 1-3 characters to SPE, wait for mailbox
 * tag bit to be  cleared and return how many bytes were written or
 * -ETIMEDOUT if transfer timed out.
 */
static int32_t combined_uart_tx(struct tegra_combined_uart_ctlr *ctlr,
		const char *buf, uint32_t num_ch, const bool flush)
{
	int32_t ret = 0;
	const uint32_t to_write = (num_ch > 3U) ? 3U : num_ch;

	/* Initialize TX value */
	uint32_t tx = (flush ? COMB_UART_PKT_FLUSH_BIT : 0U) |
			(to_write << COMB_UART_PKT_SIZE_SHIFT);

	/* Add the characters into the TX value */
	for (uint32_t i = 0; i < to_write; i++) {
		tx |= ((uint32_t)buf[i]) << (i * 8U);
	}

	/* Write to SPE mailbox */
	tegra_hsp_sm_produce(ctlr->conf.tx_hsp_id, ctlr->conf.tx_sm_num, tx);

	/* Wait for SPE mailbox to clear */
	ret = combined_uart_wait_tx_empty(ctlr);

	/* Return either error code or number of bytes written */
	if (ret == 0) {
		ret = (int32_t)to_write;
	}

	return ret;
}

static void combined_uart_hsp_sm_callback(void *data, uint32_t value)
{
	struct tegra_combined_uart_ctlr *ctlr = (struct tegra_combined_uart_ctlr *)data;
	BaseType_t yield = pdFALSE;

	uint32_t to_read = (value & COMB_UART_PKT_SIZE_MASK) >> COMB_UART_PKT_SIZE_SHIFT;

	for (uint32_t i = 0; i < to_read; i++) {
		char ch = (char)(value & 0xFFU);
		xQueueSendToBackFromISR(ctlr->rx_queue, &ch, &yield);
		value >>= 8;
	}

	/* clear interrupt */
	tegra_hsp_sm_vacate(ctlr->conf.rx_hsp_id, ctlr->conf.rx_sm_num);

	portYIELD_FROM_ISR(yield);
}

static inline int32_t combined_uart_rx(struct tegra_combined_uart_ctlr *ctlr, char *ch, TickType_t ticks_to_wait)
{
	int32_t ret = -ETIMEDOUT;

	if (xQueueReceive(ctlr->rx_queue, ch, ticks_to_wait) == pdTRUE) {
		ret = 0;
	}

	return ret;
}

int32_t tegra_combined_uart_tx_init(struct tegra_combined_uart_id *id)
{
	int32_t ret = 0;
	/* struct tegra_combined_uart_id assumed to be first field in tegra_combined_uart_ctlr */
	struct tegra_combined_uart_ctlr *ctlr = (struct tegra_combined_uart_ctlr *)id;

	if (ctlr->conf.tx_hsp_id == NULL) {
		ret = -1;
		goto exit;
	}

	ctlr->tx_enabled = true;

exit:
	return ret;
}

int32_t tegra_combined_uart_rx_init(struct tegra_combined_uart_id *id)
{
	int32_t ret = 0;
	/* struct tegra_combined_uart_id assumed to be first field in tegra_combined_uart_ctlr */
	struct tegra_combined_uart_ctlr *ctlr = (struct tegra_combined_uart_ctlr *)id;

	/* exit if rx is not to be enabled */
	if (ctlr->conf.rx_hsp_id == NULL) {
		ret = -1;
		goto exit;
	}

	ctlr->rx_queue = xQueueCreate(COMBINED_UART_RX_Q_SIZE, sizeof(char));

	if (!ctlr->rx_queue) {
		ret = -2;
		goto exit;
	}

	tegra_hsp_sm_full_enable(id->rx_hsp_id, id->rx_sm_num, combined_uart_hsp_sm_callback, (void*)id);

	ctlr->rx_enabled = true;

exit:
	return ret;
}

int32_t tegra_combined_uart_send(struct tegra_combined_uart_id *id, const char *data, uint32_t len)
{
	/* struct tegra_combined_uart_id assumed to be first field in tegra_combined_uart_ctlr */
	struct tegra_combined_uart_ctlr *ctlr = (struct tegra_combined_uart_ctlr *)id;
	uint32_t count = 0;
	int32_t ret = 0;

	if (unlikely(!ctlr->tx_enabled || data == NULL)) {
		ret = -1;
		goto exit;
	}

	if (unlikely(len == 0)) {
		ret = 0;
		goto exit;
	}

	do {
		ret = combined_uart_tx(ctlr, data, len, false);
		if (ret < 0) {
			goto exit;
		}

		len -= ret;
		count += ret;
		data += ret;
	} while (len > 0);

	/* Flush */
	ret = combined_uart_tx(ctlr, data, 0U, true);

	if (ret >= 0) {
		ret = (int32_t)count;
	}

exit:
	return ret;
}

int32_t tegra_combined_uart_recv(struct tegra_combined_uart_id *id, char *data, uint32_t len, int32_t timeout_ms)
{
	int32_t ret = 0;
	/* struct tegra_combined_uart_id assumed to be first field in tegra_combined_uart_ctlr */
	struct tegra_combined_uart_ctlr *ctlr = (struct tegra_combined_uart_ctlr *)id;
	uint32_t count = 0;

	if (unlikely(!ctlr->rx_enabled || data == NULL)) {
		ret = -1;
		goto exit;
	}

	if (unlikely(len == 0)) {
		ret = 0;
		goto exit;
	}

	TickType_t ticks_to_wait = timeout_ms / portTICK_PERIOD_MS;

	if (timeout_ms == CUART_RX_TIMEOUT_NEVER) {
		ticks_to_wait = portMAX_DELAY;
	}
	else if (timeout_ms == CUART_RX_TIMEOUT_DEFAULT) {
		ticks_to_wait = COMB_UART_RX_TIMEOUT_TICKS;
	}

	while (true) {
		char ch = 0;

		ret = combined_uart_rx(ctlr, &ch, ticks_to_wait);

		if (ret < 0) {
			goto exit;
		}

		data[count] = ch;
		count++;

		if (len == count) {
			goto exit;
		}
	}

exit:
	if (count > 0) {
		ret = (int32_t)count;
	}
	return ret;
}
