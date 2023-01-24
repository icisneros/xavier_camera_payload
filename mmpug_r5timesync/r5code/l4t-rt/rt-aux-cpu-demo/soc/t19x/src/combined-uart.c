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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

#include <barriers.h>
#include <err-hook.h>
#include <delay.h>
#include <debug.h>
#include <irqs.h>
#include <macros.h>
#include <hsp-tegra.h>
#include <hsp-tegra-hw.h>
#include <tke-tegra.h>
#include <uart-tegra.h>

#include <mbox-aon.h>
#include <combined-uart.h>
#include <combined-uart-priv.h>

#include "config.h"

#define NUM_BYTES_FIELD_BIT	24
#define FLUSH_BIT		26
#define HW_FLUSH_BIT		27
#define DMA_MODE_BIT		30
#define INTR_TRIGGER_BIT	31
#define NUM_BYTES(reg_val)	((reg_val >> NUM_BYTES_FIELD_BIT) & 0x3)

#define TASK_NAME_LEN		32

#define RX_TIMEOUT_MS		100

#define TAG_ESCAPE_CHAR		0xff
#define TAG_ESCAPE_STRING	"\377"

#define TAG_RESET		0xfd
#define TAG_BOOT_RESET_STRING	"\0\377\375"

#if defined(ENABLE_COMBINED_UART_SERVER)

static SemaphoreHandle_t tx_mutex;

static bool combined_uart_inited = false;
static char spe_msg_buf[SPE_NUM_MSGS][SPE_MSG_SIZE];
static unsigned int spe_msg_buf_read_idx = 0, spe_msg_buf_write_idx = 0;
extern struct ext_client_id ext_client_ids[COMBINED_UART_NUM_EXT_CLIENTS];

static struct tegra_uart_id *combined_uart_id = NULL;
static struct tegra_uart_conf combined_uart_conf = {
	.parity = TEGRA_UART_NO_PARITY,
	.stop_bits = TEGRA_UART_STOP_BITS_1,
	.data_bits = TEGRA_UART_DATA_BITS_8,
	.baud = 115200,
};

static struct ext_client_id spe_id = {
	.tag = 0xe0,
};

static const struct ext_client_id *last_id = NULL;

static void combined_uart_process_tx_queue(struct ext_client_id *id, uint32_t msg);
static void print_msg(const struct ext_client_id *id, const char *msg_buf, size_t len);

int _write(int file, char *ptr, int len);

static void combined_uart_raw_print(const char *str, size_t len)
{
	size_t written = 0;

	if (!combined_uart_inited)
		return;

	if (in_interrupt() || in_critical()) {
		tegra_uart_write_now(combined_uart_id, str, len);
		return;
	}

	written = 0;
	while (written < len)
		written += tegra_uart_write(combined_uart_id, str,
				len - written, portMAX_DELAY);
}

static void combined_uart_dbg_putc(const char *str, size_t len)
{
	const char *buf, *tag_escape;
	size_t remaining = len;
	size_t current_len;

	buf = str;

	while (remaining) {
		tag_escape = memchr(buf, TAG_ESCAPE_CHAR, remaining);

		if (tag_escape)
			current_len = tag_escape - buf + 1;
		else
			current_len = remaining;

		combined_uart_raw_print(buf, current_len);
		if (buf[current_len - 1] == TAG_ESCAPE_CHAR)
			combined_uart_raw_print(TAG_ESCAPE_STRING, 1);

		buf += current_len;
		remaining -= current_len;
	}
}

static inline bool msg_needs_deferred_ack(uint32_t msg)
{
	return (msg & (BIT(FLUSH_BIT) | BIT(HW_FLUSH_BIT)));
}

/*
 * This is the entry point for external (non-SPE) clients for sending message
 * packets to the combined UART server.
 */
static void combined_uart_irq(void *param, uint32_t data)
{
	struct ext_client_id *id = param;
	BaseType_t higher_prio_task_woken = pdFALSE;

	id->rx_enabled = true;

	data &= ~MBOX_TAG_BIT;
	xQueueSendToBackFromISR(id->tx_queue, &data, &higher_prio_task_woken);
	portYIELD_FROM_ISR(higher_prio_task_woken);

	if (msg_needs_deferred_ack(data)) {
		/* Pause until this message is processed */
		tegra_hsp_sm_full_disable(&tegra_hsp_id_aon, id->recv_mbox);
	} else if (xQueueIsQueueFullFromISR(id->tx_queue)) {
		/* Pause until the queue has space */
		id->tx_queue_full = true;
		tegra_hsp_sm_full_disable(&tegra_hsp_id_aon, id->recv_mbox);
	} else {
		/* Continue normally */
		tegra_hsp_sm_vacate(&tegra_hsp_id_aon, id->recv_mbox);
	}
}

int _write(int file, char *ptr, int len)
{
	if (len <= 0)
		return len;

	if (file == 2) {
		print_msg(&spe_id, ptr, (size_t)len);
		return len;
	}

	if (file != 1)
		return -1;

	if (len > SPE_MSG_SIZE)
		len = SPE_MSG_SIZE;

	//XXX: Can potentially overwrite messages
	taskENTER_CRITICAL();
	memcpy(spe_msg_buf[spe_msg_buf_write_idx], ptr, len);
	spe_msg_buf_write_idx = (spe_msg_buf_write_idx + 1) % SPE_NUM_MSGS;
	taskEXIT_CRITICAL();

	if (combined_uart_inited)
		xQueueSendToBack(spe_id.tx_queue, &len, portMAX_DELAY);
	else
		xQueueSendToBack(spe_id.tx_queue, &len, 0);

	return len;
}

static void print_msg(const struct ext_client_id *id, const char *msg_buf, size_t len)
{
	char c;

	if (last_id != id) {
		combined_uart_raw_print(TAG_ESCAPE_STRING, 1);
		c = id->tag;
		combined_uart_raw_print(&c, 1);
		last_id = id;
	}

	if (len > 0)
		combined_uart_dbg_putc(msg_buf, len);
}

/*
 * This is the function which processes the messages from all combined UART
 * clients.
 */
static void combined_uart_process_tx_queue(struct ext_client_id *id, uint32_t msg)

{
	unsigned int i;
	unsigned int num_bytes = NUM_BYTES(msg);
	char buf[3];

	/* Extract characters from message packet */
	for (i = 0; i < num_bytes; i++)
		buf[i] = (msg >> i*8) & 0xff;

	xSemaphoreTake(tx_mutex, portMAX_DELAY);
	print_msg(id, buf, num_bytes);
	/* Handle hardware flush command */
	if (msg & BIT(HW_FLUSH_BIT)) {
		tegra_uart_flush_tx_buffer(combined_uart_id, 0);
		tegra_uart_flush_tx_hw_fifo(combined_uart_id);
	}
	xSemaphoreGive(tx_mutex);
}

/*
 * This is the loop for the combined UART server task. The task waits for and
 * processes messages from combined UART clients.
 */
static portTASK_FUNCTION(combined_uart_tx_task, pvParameters)
{
	struct ext_client_id *id = (struct ext_client_id *)pvParameters;
	uint32_t msg;

	for (;;) {
		xQueueReceive(id->tx_queue, &msg, portMAX_DELAY);
		combined_uart_process_tx_queue(id, msg);
		if ((id->tx_queue_full && uxQueueSpacesAvailable(id->tx_queue) != 0) ||
				msg_needs_deferred_ack(msg)) {
			/*
			 * Entering this block implies that the mbox IRQ is disabled.
			 * Therefore, synchronization is not necessary.
			 */
			id->tx_queue_full = false;
			tegra_hsp_sm_vacate(&tegra_hsp_id_aon, id->recv_mbox);
			tegra_hsp_sm_full_enable(&tegra_hsp_id_aon, id->recv_mbox,
				combined_uart_irq, id);
		}
	}
}

static void send_rx_chars(struct ext_client_id *id, unsigned char c[], int len)
{
	uint32_t mbox_val;
	uint32_t time;
	int i;

	if (len > 3)
		len = 3;
	if (len <= 0)
		return;

	mbox_val = ((len & 0x3) << NUM_BYTES_FIELD_BIT) | BIT(INTR_TRIGGER_BIT);
	for (i = 0; i < len; i++) {
		mbox_val |= ((c[i] & 0xff) << (i * 8));
	}
	time = tegra_tke_get_usec();
	while (tegra_hsp_sm_peek(id->hsp_id, id->send_mbox) >= 0) {
		if (id->rx_timedout)
			return;
		if (tegra_tke_get_usec() - time > RX_TIMEOUT_MS * 1000) {
			id->rx_timedout = true;
			return;
		}
	}
	id->rx_timedout = false;
	tegra_hsp_sm_produce(id->hsp_id, id->send_mbox, mbox_val);
}

static portTASK_FUNCTION(combined_uart_rx_task, pvParameters)
{
	struct ext_client_id *id = (struct ext_client_id *)pvParameters;
	unsigned char chs[3];
	unsigned char c;
	int len;

	for(;;) {
		len = 0;
		xQueueReceive(id->rx_queue, &c, portMAX_DELAY);
		chs[len++] = c;
		if (xQueueReceive(id->rx_queue, &c, 0) == pdTRUE)
			chs[len++] = c;
		if (xQueueReceive(id->rx_queue, &c, 0) == pdTRUE)
			chs[len++] = c;
		if (id->rx_enabled)
			send_rx_chars(id, chs, len);
	}
}

static portTASK_FUNCTION(combined_uart_tx_task_spe, pvParameters)
{
	struct ext_client_id *id = (struct ext_client_id *)pvParameters;
	uint32_t len;

	tegra_uart_flush_tx_hw_fifo(combined_uart_id);
	for (;;) {
		xQueueReceive(id->tx_queue, &len, portMAX_DELAY);
		xSemaphoreTake(tx_mutex, portMAX_DELAY);
		print_msg(id, spe_msg_buf[spe_msg_buf_read_idx], len);
		spe_msg_buf_read_idx = (spe_msg_buf_read_idx + 1) % SPE_NUM_MSGS;
		xSemaphoreGive(tx_mutex);
	}
}

static portTASK_FUNCTION(combined_uart_rx_task_spe, pvParameters)
{
	struct ext_client_id *id = (struct ext_client_id *)pvParameters;
	struct ext_client_id *dest_id = &ext_client_ids[COMBINED_UART_CCPLEX];
	(void)id;
	bool in_escape = false;
	char c;
	int i;

	for(;;) {
		if (!tegra_uart_read(combined_uart_id, &c, 1, portMAX_DELAY))
			continue;

		if (!in_escape) {
			if (c == TAG_ESCAPE_CHAR)
				in_escape = true;
			else if (dest_id->rx_queue != NULL)
				xQueueSend(dest_id->rx_queue, &c, 0);
			continue;
		}

		in_escape = false;

		/* Handle escape character */
		if (c == TAG_ESCAPE_CHAR) {
			if (dest_id->rx_queue != NULL)
				xQueueSend(dest_id->rx_queue, &c, 0);
			continue;
		}

		if (c == TAG_RESET) {
			xSemaphoreTake(tx_mutex, portMAX_DELAY);
			last_id = NULL;
			xSemaphoreGive(tx_mutex);
			continue;
		}

		/* void the input */
		if (c == spe_id.tag)
			dest_id = &spe_id;

		for (i = 0; i < COMBINED_UART_NUM_EXT_CLIENTS; i++) {
			if (c == ext_client_ids[i].tag) {
				dest_id = &ext_client_ids[i];
				break;
			}
		}
	}
}

int combined_uart_preinit(void)
{
	spe_id.tx_queue = xQueueCreate(SPE_NUM_MSGS, sizeof(uint32_t));
	if (!spe_id.tx_queue)
		return -1;
	return 0;
}

/*
 * Driver initialization function
 */
int combined_uart_init(struct tegra_uart_id *uart_id)
{
	char buf[TASK_NAME_LEN];
	int i;
	struct ext_client_id *id;

	combined_uart_id = uart_id;
	tegra_uart_init(uart_id, &combined_uart_conf);
	combined_uart_inited = true;

	combined_uart_raw_print(TAG_BOOT_RESET_STRING, 3);

	tx_mutex = xSemaphoreCreateMutex();
	if (!tx_mutex) {
		error_hook("couldn't create mutex");
		return -1;
	}

	xSemaphoreGive(tx_mutex);

	if (xTaskCreate(combined_uart_tx_task_spe, "combined-uart-tx-spe",
				configMINIMAL_STACK_SIZE, &spe_id,
				configMAX_PRIORITIES - 3, &spe_id.tx_task) != pdPASS) {
		error_hook("couldn't create SPE tx task");
		return -1;
	}

	/* We keep the SPE TX task and mutex even on failure. */
	if (xTaskCreate(combined_uart_rx_task_spe, "combined-uart-rx-spe",
				configMINIMAL_STACK_SIZE, &spe_id,
				configMAX_PRIORITIES - 2, &spe_id.rx_task) != pdPASS) {
		error_hook("couldn't create SPE rx task");
		return -1;
	}


	for (i = 0; i < COMBINED_UART_NUM_EXT_CLIENTS; i++) {
		id = &ext_client_ids[i];
		if (!id->enabled)
			continue;

		id->rx_enabled = false;
		id->rx_timedout = false;
		id->tx_queue_full = false;

		id->tx_queue = xQueueCreate(EXT_CLIENT_MSG_SIZE, sizeof(uint32_t));
		if (!id->tx_queue)
			goto fail_print_msg;

		if (id->hsp_id != NULL) {
			/* Supportx RX */
			id->rx_queue = xQueueCreate(EXT_CLIENT_RX_BUF_SIZE, 1);
			if (!id->rx_queue)
				goto fail_tx_queue;

			id->rx_ack_sem = xSemaphoreCreateBinary();
			if (!id->rx_ack_sem)
				goto fail_rx_queue;
			xSemaphoreGive(id->rx_ack_sem);

			snprintf(buf, sizeof(buf), "combined-uart-rx-%s", id->name);
			if (xTaskCreate(combined_uart_rx_task, buf, configMINIMAL_STACK_SIZE,
					id, configMAX_PRIORITIES - 6, &id->rx_task) != pdPASS)
				goto fail_sem;
		}

		snprintf(buf, sizeof(buf), "combined-uart-tx-%s", id->name);
		if (xTaskCreate(combined_uart_tx_task, buf, configMINIMAL_STACK_SIZE,
				id, configMAX_PRIORITIES - 5, &id->tx_task) != pdPASS)
			goto fail_rx_task;

		tegra_hsp_sm_full_enable(&tegra_hsp_id_aon, id->recv_mbox,
			combined_uart_irq, id);
		continue;

fail_rx_task:
		if (id->rx_task != NULL)
			vTaskDelete(id->rx_task);
		id->rx_task = NULL;
fail_sem:
		if (id->rx_ack_sem != NULL)
			vSemaphoreDelete(id->rx_ack_sem);
		id->rx_ack_sem = NULL;
fail_rx_queue:
		if (id->rx_queue != NULL)
			vQueueDelete(id->rx_queue);
		id->rx_queue = NULL;
fail_tx_queue:
		vQueueDelete(id->tx_queue);
		id->tx_queue = NULL;
fail_print_msg:
		error_hookf("couldn't allocate memory for %s", id->name);
		break;
	}

	return 0;
}

void combined_uart_suspend(void)
{
	struct ext_client_id *bpmp_id;

	if (!combined_uart_id)
		return;

	bpmp_id = &ext_client_ids[COMBINED_UART_BPMP];
	xSemaphoreTake(tx_mutex, portMAX_DELAY);
	print_msg(bpmp_id, "", 0);
	combined_uart_inited = false;
	xSemaphoreGive(tx_mutex);

	tegra_uart_flush_tx_buffer(combined_uart_id, 0);
	tegra_uart_flush_tx_hw_fifo(combined_uart_id);
}

void combined_uart_resume(void)
{
	if (!combined_uart_id)
		return;

	tegra_uart_init_hw(combined_uart_id, &combined_uart_conf);
	combined_uart_inited = true;
}

#endif
