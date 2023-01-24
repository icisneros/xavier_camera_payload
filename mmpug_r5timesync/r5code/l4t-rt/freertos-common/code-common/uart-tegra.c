/*
* Copyright (c) 2014-2017 NVIDIA CORPORATION.  All rights reserved.
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

#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <queue.h>
#include <task.h>

#include <aruart.h>

#include <delay.h>
#include <reg-access.h>
#include <irqs.h>
#include <clk-tegra.h>

#include <uart-tegra-priv.h>

#define TEGRA_UART_LCR_DLAB		(UART_LCR_0_DLAB_ENABLE << UART_LCR_0_DLAB_SHIFT)
#define TEGRA_UART_IER_THRE		(UART_IER_DLAB_0_0_IE_THR_ENABLE << UART_IER_DLAB_0_0_IE_THR_SHIFT)
#define TEGRA_UART_LSR_RDY		(UART_LSR_0_RDR_DATA_IN_FIFO << UART_LSR_0_RDR_SHIFT)
#define TEGRA_UART_LSR_TX_FIFO_FULL	(UART_LSR_0_TX_FIFO_FULL_FULL << UART_LSR_0_TX_FIFO_FULL_SHIFT)
#define TEGRA_UART_LSR_THRE_EMPTY	(UART_LSR_0_THRE_EMPTY << UART_LSR_0_THRE_SHIFT)
#define TEGRA_UART_LSR_TX_SHIFT_EMPTY	(UART_LSR_0_TMTY_EMPTY << UART_LSR_0_TMTY_SHIFT)
#define TEGRA_UART_IIR_NO_INT		(UART_IIR_FCR_0_IS_STA_NO_INTR_PEND << UART_IIR_FCR_0_IS_STA_SHIFT)

#define TEGRA_UART_QUEUE_SIZE		256

static inline uint32_t tegra_uart_readl(struct tegra_uart_ctlr *ctlr,
	uint32_t offset)
{
	offset += ctlr->id.base_addr;
	return readl(offset);
}

static inline void tegra_uart_writel(struct tegra_uart_ctlr *ctlr,
	uint32_t val, uint32_t offset)
{
	offset += ctlr->id.base_addr;
	writel(val, offset);
}

int tegra_uart_init_hw(struct tegra_uart_id *id, struct tegra_uart_conf *conf)
{
	struct tegra_uart_ctlr *ctlr = (struct tegra_uart_ctlr *)id;
	uint32_t fcr, divisor, comm_param, ier;

	tegra_clk_enable(ctlr->id.clk);
	tegra_clk_reset_pulse(ctlr->id.reset, 10);

	/* Program the FCR register */
	fcr = (UART_IIR_FCR_0_FCR_EN_FIFO_ENABLE << UART_IIR_FCR_0_FCR_EN_FIFO_SHIFT);
	fcr |= (UART_IIR_FCR_0_TX_TRIG_FIFO_COUNT_GREATER_8 <<
			UART_IIR_FCR_0_TX_TRIG_SHIFT);
	fcr |= (UART_IIR_FCR_0_RX_TRIG_FIFO_COUNT_GREATER_4 <<
			UART_IIR_FCR_0_RX_TRIG_SHIFT);
	tegra_uart_writel(ctlr, fcr, UART_IIR_FCR_0);

	tegra_clk_set_rate((ctlr->id.clk), conf->baud * 16);
	divisor = 1;

	switch (conf->parity) {
	case TEGRA_UART_NO_PARITY:
		comm_param = (UART_LCR_0_PAR_NO_PARITY << UART_LCR_0_PAR_SHIFT);
		break;
	case TEGRA_UART_ODD_PARITY:
		comm_param = (UART_LCR_0_EVEN_DISABLE << UART_LCR_0_EVEN_SHIFT) |
			(UART_LCR_0_PAR_PARITY << UART_LCR_0_PAR_SHIFT);
		break;
	case TEGRA_UART_EVEN_PARITY:
		comm_param = (UART_LCR_0_EVEN_ENABLE << UART_LCR_0_EVEN_SHIFT) |
			(UART_LCR_0_PAR_PARITY << UART_LCR_0_PAR_SHIFT);
		break;
	default:
		goto conf_err;
	}

	switch (conf->data_bits) {
	case TEGRA_UART_DATA_BITS_5:
		comm_param |= (UART_LCR_0_WD_SIZE_WORD_LENGTH_5 <<
				UART_LCR_0_WD_SIZE_SHIFT);
		break;
	case TEGRA_UART_DATA_BITS_6:
		comm_param |= (UART_LCR_0_WD_SIZE_WORD_LENGTH_6 <<
				UART_LCR_0_WD_SIZE_SHIFT);
		break;
	case TEGRA_UART_DATA_BITS_7:
		comm_param |= (UART_LCR_0_WD_SIZE_WORD_LENGTH_7 <<
				UART_LCR_0_WD_SIZE_SHIFT);
		break;
	case TEGRA_UART_DATA_BITS_8:
		comm_param |= (UART_LCR_0_WD_SIZE_WORD_LENGTH_8 <<
				UART_LCR_0_WD_SIZE_SHIFT);
		break;
	default:
		goto conf_err;
	}

	if (conf->stop_bits == TEGRA_UART_STOP_BITS_2)
		comm_param |= (UART_LCR_0_STOP_ENABLE << UART_LCR_0_STOP_SHIFT);

	tegra_uart_writel(ctlr, 0x0, UART_LCR_0);
	tegra_uart_writel(ctlr, 0x0, UART_IER_DLAB_0_0);

	tegra_uart_writel(ctlr, comm_param | TEGRA_UART_LCR_DLAB, UART_LCR_0);
	tegra_uart_writel(ctlr, divisor & 0xFF, UART_THR_DLAB_0_0);
	tegra_uart_writel(ctlr, ((divisor >> 8) & 0xFF), UART_IER_DLAB_0_0);
	tegra_uart_writel(ctlr, comm_param & ~TEGRA_UART_LCR_DLAB, UART_LCR_0);

	/* dummy read to ensure write is posted */
	tegra_uart_readl(ctlr, UART_SPR_0);

	/* Enable RX Interrupts */
	ier = (UART_IER_DLAB_0_0_IE_RHR_ENABLE << UART_IER_DLAB_0_0_IE_RHR_SHIFT) |
		(UART_IER_DLAB_0_0_IE_RXS_ENABLE << UART_IER_DLAB_0_0_IE_RXS_SHIFT) |
		(UART_IER_DLAB_0_0_IE_RX_TIMEOUT_ENABLE << UART_IER_DLAB_0_0_IE_RX_TIMEOUT_SHIFT);
	tegra_uart_writel(ctlr, ier, UART_IER_DLAB_0_0);

	irq_set_handler(ctlr->id.irq, tegra_uart_irq, ctlr);
	irq_enable(ctlr->id.irq);

	return 0;

conf_err:
	tegra_clk_disable(ctlr->id.clk);
	return 1;
}

int tegra_uart_init(struct tegra_uart_id *id, struct tegra_uart_conf *conf)
{
	struct tegra_uart_ctlr *ctlr = (struct tegra_uart_ctlr *)id;

	ctlr->tx_queue = xQueueCreate(TEGRA_UART_QUEUE_SIZE, sizeof(char));
	if (!ctlr->tx_queue)
		return 1;
	ctlr->rx_queue = xQueueCreate(TEGRA_UART_QUEUE_SIZE, sizeof(char));
	if (!ctlr->rx_queue) {
		vQueueDelete(ctlr->tx_queue);
		return 1;
	}
	ctlr->tx_sem = xSemaphoreCreateBinary();
	if (!ctlr->tx_sem) {
		vQueueDelete(ctlr->tx_queue);
		vQueueDelete(ctlr->rx_queue);
		return 1;
	}
	xSemaphoreGive(ctlr->tx_sem);

	if (tegra_uart_init_hw(id, conf)) {
		vSemaphoreDelete(ctlr->tx_sem);
		vQueueDelete(ctlr->tx_queue);
		vQueueDelete(ctlr->rx_queue);
		return 1;
	}

	return 0;
}

void tegra_uart_irq(void *data)
{
	struct tegra_uart_id *id = data;
	struct tegra_uart_ctlr *ctlr = (struct tegra_uart_ctlr *)id;
	uint32_t iir, ier;
	char c_in, c_out;
	portBASE_TYPE higher_prio_task_woken = pdFALSE;

	while (1) {
		iir = tegra_uart_readl(ctlr, UART_IIR_FCR_0) & 0x0f;
		if (iir & TEGRA_UART_IIR_NO_INT)
			break;

		switch(iir) {
		case 0x0c: /* RX Timeout */
		case 0x04: /* Receive */
			do {
				c_in = tegra_uart_readl(ctlr, UART_THR_DLAB_0_0) & 0xff;
				xQueueSendFromISR(ctlr->rx_queue, &c_in,
						&higher_prio_task_woken);
			} while(tegra_uart_readl(ctlr, UART_LSR_0) & TEGRA_UART_LSR_RDY);
			break;
		case 0x02: /* THRE */
			do {
				if (xQueueReceiveFromISR(ctlr->tx_queue, &c_out,
							&higher_prio_task_woken) != pdTRUE) {
					ier = tegra_uart_readl(ctlr, UART_IER_DLAB_0_0);
					tegra_uart_writel(ctlr, ier & ~TEGRA_UART_IER_THRE, UART_IER_DLAB_0_0);
					break;
				}
				tegra_uart_writel(ctlr, c_out, UART_THR_DLAB_0_0);
			} while (tegra_uart_readl(ctlr, UART_LSR_0) & TEGRA_UART_LSR_THRE_EMPTY);
			break;
		case 0x06: /* LSINT */
			tegra_uart_readl(ctlr, UART_LSR_0);
			break;
		case 0x00: /* MSINT */
			tegra_uart_readl(ctlr, UART_MSR_0);
			break;
		}
	}

	portYIELD_FROM_ISR(higher_prio_task_woken);
}

static TickType_t tegra_uart_timeout_us_to_ticks(uint64_t timeout)
{
	if (timeout >= portMAX_DELAY)
		return portMAX_DELAY;
	/* Assuming that the tick rate is less than 1 MHz */
	return timeout * configTICK_RATE_HZ / 1000000;
}

static TickType_t tegra_uart_get_elapsed_ticks(uint64_t tstart, uint64_t timeout)
{
	uint64_t tcurr = get_time_ticks();

	if (timeout >= portMAX_DELAY)
		return portMAX_DELAY;
	else if ((tcurr - tstart) >= timeout)
		return 0;
	else
		return tegra_uart_timeout_us_to_ticks(timeout - (tcurr - tstart));
}

int tegra_uart_read(struct tegra_uart_id *id, char *buf, int count,
	uint64_t timeout)
{
	struct tegra_uart_ctlr *ctlr = (struct tegra_uart_ctlr *)id;
	uint64_t tstart;
	TickType_t timeout_left;
	int read_cnt;

	timeout_left = tegra_uart_timeout_us_to_ticks(timeout);
	tstart = get_time_ticks();

	for (read_cnt = 0; read_cnt < count; read_cnt++) {
		if (xQueueReceive(ctlr->rx_queue, buf,
					timeout_left) == pdFALSE)
			break;
		buf++;
		timeout_left = tegra_uart_get_elapsed_ticks(tstart, timeout);
	}

	return read_cnt;
}

static void tegra_uart_disable_thre_interrupt(struct tegra_uart_ctlr *ctlr)
{
	uint32_t ier;
	irq_disable(ctlr->id.irq);
	ier = tegra_uart_readl(ctlr, UART_IER_DLAB_0_0);
	if (ier & TEGRA_UART_IER_THRE)
		tegra_uart_writel(ctlr, ier & ~TEGRA_UART_IER_THRE,
				UART_IER_DLAB_0_0);
	irq_enable(ctlr->id.irq);
}

static void tegra_uart_enable_thre_interrupt(struct tegra_uart_ctlr *ctlr)
{
	uint32_t ier;

	irq_disable(ctlr->id.irq);
	ier = tegra_uart_readl(ctlr, UART_IER_DLAB_0_0);
	if (!(ier & TEGRA_UART_IER_THRE))
		tegra_uart_writel(ctlr, ier | TEGRA_UART_IER_THRE,
				UART_IER_DLAB_0_0);
	irq_enable(ctlr->id.irq);
}

int tegra_uart_write(struct tegra_uart_id *id, const char *buf, int count,
	uint64_t timeout)
{
	struct tegra_uart_ctlr *ctlr = (struct tegra_uart_ctlr *)id;
	uint64_t tstart;
	TickType_t timeout_left;
	int write_cnt;

	timeout_left = tegra_uart_timeout_us_to_ticks(timeout);
	tstart = get_time_ticks();

	if (xSemaphoreTake(ctlr->tx_sem, timeout_left) != pdTRUE)
		return 0;

	for (write_cnt = 0; write_cnt < count; write_cnt++) {
		timeout_left = tegra_uart_get_elapsed_ticks(tstart, timeout);
		if (uxQueueSpacesAvailable(ctlr->tx_queue) == 0)
			tegra_uart_enable_thre_interrupt(ctlr);

		if (xQueueSend(ctlr->tx_queue, buf,
					timeout_left) != pdTRUE)
			break;
		buf++;
	}

	if (write_cnt)
		tegra_uart_enable_thre_interrupt(ctlr);

	xSemaphoreGive(ctlr->tx_sem);

	return write_cnt;
}

int tegra_uart_flush_tx_buffer(struct tegra_uart_id *id, BaseType_t timeout)
{
	struct tegra_uart_ctlr *ctlr = (struct tegra_uart_ctlr *)id;
	char c;

	if (xSemaphoreTake(ctlr->tx_sem, timeout) != pdTRUE)
		return -1;
	tegra_uart_disable_thre_interrupt(ctlr);
	while (xQueueReceive(ctlr->tx_queue, &c, 0) == pdTRUE) {
		while (!(tegra_uart_readl(ctlr, UART_LSR_0) & TEGRA_UART_LSR_THRE_EMPTY))
			;
		tegra_uart_writel(ctlr, c, UART_THR_DLAB_0_0);
	}
	xSemaphoreGive(ctlr->tx_sem);
	return 0;
}

void tegra_uart_flush_tx_hw_fifo(struct tegra_uart_id *id)
{
	struct tegra_uart_ctlr *ctlr = (struct tegra_uart_ctlr *)id;

	while (!(tegra_uart_readl(ctlr, UART_LSR_0) & TEGRA_UART_LSR_TX_SHIFT_EMPTY))
		;
}

void tegra_uart_write_now(struct tegra_uart_id *id, const char *buf, int count)
{
	struct tegra_uart_ctlr *ctlr = (struct tegra_uart_ctlr *)id;
	int i;

	for (i = 0; i < count; i++) {
		while (!(tegra_uart_readl(ctlr, UART_LSR_0) & TEGRA_UART_LSR_THRE_EMPTY))
			;
		tegra_uart_writel(ctlr, *buf++, UART_THR_DLAB_0_0);
	}
}
