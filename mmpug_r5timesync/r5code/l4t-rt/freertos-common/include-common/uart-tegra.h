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

#ifndef _UART_TEGRA_H_
#define _UART_TEGRA_H_

enum tegra_uart_parity {
	TEGRA_UART_NO_PARITY,
	TEGRA_UART_ODD_PARITY,
	TEGRA_UART_EVEN_PARITY,
};

enum tegra_uart_stop_bits {
	TEGRA_UART_STOP_BITS_1,
	TEGRA_UART_STOP_BITS_2
};

enum tegra_uart_data_bits {
	TEGRA_UART_DATA_BITS_5,
	TEGRA_UART_DATA_BITS_6,
	TEGRA_UART_DATA_BITS_7,
	TEGRA_UART_DATA_BITS_8,
} tegra_uart_data_bits;

struct tegra_uart_conf {
	enum tegra_uart_parity parity;
	enum tegra_uart_stop_bits stop_bits;
	enum tegra_uart_data_bits data_bits;
	unsigned int baud;
};

struct tegra_uart_id;

int tegra_uart_init_hw(struct tegra_uart_id *id, struct tegra_uart_conf *conf);
int tegra_uart_init(struct tegra_uart_id *id, struct tegra_uart_conf *conf);
int tegra_uart_write(struct tegra_uart_id *id, const char *buf, int count, uint64_t timeout);
int tegra_uart_read(struct tegra_uart_id *id, char *buf, int count, uint64_t timeout);
void tegra_uart_irq(void *data);
int tegra_uart_flush_tx_buffer(struct tegra_uart_id *id, BaseType_t timeout);
void tegra_uart_flush_tx_hw_fifo(struct tegra_uart_id *id);
void tegra_uart_write_now(struct tegra_uart_id *id, const char *buf, int count);

#endif
