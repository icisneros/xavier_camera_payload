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

#ifndef _COMBINED_UART_H_
#define _COMBINED_UART_H_

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

#include <stdbool.h>
#include <stdint.h>

struct tegra_hsp_id;
struct tegra_uart_id;

struct ext_client_id {
	const struct tegra_hsp_id *hsp_id;
	const char *name;
	QueueHandle_t tx_queue;
	TaskHandle_t tx_task;
	QueueHandle_t rx_queue;
	TaskHandle_t rx_task;
	SemaphoreHandle_t send_mbox_mutex;
	SemaphoreHandle_t rx_ack_sem;
	uint32_t recv_mbox;
	uint32_t send_mbox;
	uint8_t tag;
	bool enabled;
	bool tx_queue_full;
	bool rx_enabled;
	bool rx_timedout;
};

int combined_uart_preinit(void);
int combined_uart_init(struct tegra_uart_id *uart_id);
void combined_uart_suspend(void);
void combined_uart_resume(void);

#endif /* _COMBINED_UART_H_ */
