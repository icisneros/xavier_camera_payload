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

#ifndef _SPI_TEGRA_PRIV_H_
#define _SPI_TEGRA_PRIV_H_

#include <stdint.h>
#include <stdbool.h>

#include <FreeRTOS.h>
#include <queue.h>

#include <clk-tegra.h>
#include <spi-tegra.h>
#include <tegra-gpcdma.h>

#define MAX_CHIP_SELECT 4

struct tegra_spi_id {
	const char *devname;
	uint32_t base_addr;
	const struct tegra_clk *div_clk;
	const struct tegra_rst *reset;
	uint32_t irq;
};

struct tegra_spi_ctlr {
	/*
	 * The driver relies on this being the first field, since it casts
	 * pointers between structs tegra_spi_id and tegra_spi_data
	 */
	struct tegra_spi_id conf;

	QueueHandle_t irq_queue;
	QueueHandle_t dma_queue;

	struct tegra_gpcdma_id *dma_id;
	struct {
		int tx, rx;
	} dma_channel;
	uint8_t dma_slave_req;

	uint32_t spi_clk_rate;

	uint32_t bits_per_word;

	uint32_t cur_pos;
	uint32_t words_per_32bit;
	uint32_t bytes_per_word;
	uint32_t curr_dma_words;
	uint32_t cur_direction;

	uint32_t cur_rx_pos;
	uint32_t cur_tx_pos;

	uint32_t dma_buf_size;
	uint32_t max_buf_size;
	bool is_curr_dma_xfer;
	bool is_hw_based_cs;

	uint32_t tx_status;
	uint32_t rx_status;
	uint32_t status_reg;
	bool is_packed;

	uint32_t cmd1_reg_val;
	uint32_t dma_ctrl_reg_val;
	uint32_t def_cmd1_reg_val;
	uint32_t def_cmd2_reg_val;
	uint32_t spi_cs_timing;
	uint8_t def_chip_select;

	struct tegra_spi_client_setup cdata[MAX_CHIP_SELECT];

	struct tegra_spi_xfer *curr_xfer;

	uint32_t *rx_dma_buf;
	uint32_t *tx_dma_buf;
	uint8_t dma_status;
	bool busy;
};

#endif
