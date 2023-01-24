/*
 * Copyright (c) 2019, NVIDIA CORPORATION.  All rights reserved.
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

/* Note: Follow doc/uart.txt to successfully run this demo app */

#include <FreeRTOS.h>

#include <err-hook.h>
#include <task.h>
#include <string.h>
#include <stdio.h>

#include "config.h"
#include "uart-tegra.h"
#include "uart-tegra-hw.h"
#include "uart-app.h"

#define UART_SET_DELAY	3000

static portTASK_FUNCTION(uart_tx_task, pvParameters)
{
	char tx_message[] = "Message from SPE R5 UART\r\n";
	int tx_len = sizeof(tx_message) - 1;

	(void)pvParameters;

	for (;;) {
		tegra_uart_write_now(&UART_APP_PORT, tx_message, tx_len);
		vTaskDelay(UART_SET_DELAY);
	};
}

static portTASK_FUNCTION(uart_rx_task, pvParameters)
{
	(void)pvParameters;

	for (;;) {
		char rx_char;
		int rx_len;

		rx_len = tegra_uart_read(&UART_APP_PORT, &rx_char, 1,
			 UART_SET_DELAY);
		if (rx_len > 0)
			printf("%c", rx_char);
	};
}

void uart_app_init(void)
{
	int ret;
	struct tegra_uart_conf uart_conf = {
		.parity = TEGRA_UART_NO_PARITY,
		.stop_bits = TEGRA_UART_STOP_BITS_1,
		.data_bits = TEGRA_UART_DATA_BITS_8,
		.baud = 115200,
	};

	ret = tegra_uart_init(&UART_APP_PORT, &uart_conf);
	if (ret != 0)
		error_hook("Fail to initialize UART port\r\n");

	ret = xTaskCreate(uart_tx_task, "uart_tx_task", 512, NULL,
		tskIDLE_PRIORITY, NULL);
	if (ret != pdPASS)
		error_hook("xTaskCreate for uart_tx_task failed\r\n");

	ret = xTaskCreate(uart_rx_task, "uart_rx_task", 512, NULL,
		tskIDLE_PRIORITY, NULL);
	if (ret != pdPASS)
		error_hook("xTaskCreate for uart_rx_task failed\r\n");
}
