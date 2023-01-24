/*
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _I2C_TEGRA_PRIV_H_
#define _I2C_TEGRA_PRIV_H_

#include <stdint.h>
#include <stdbool.h>

#include <FreeRTOS.h>
#include <queue.h>

#include <clk-tegra.h>
#include <i2c-tegra.h>

#ifdef _NV_BUILD_FPGA_
#ifndef _I2C_TEGRA_DISABLE_CAR_ACCESS
#define _I2C_TEGRA_DISABLE_CAR_ACCESS
#endif
#endif

struct tegra_i2c_id {
	const char *devname;
	uint32_t base_addr;
	uint32_t offset_mult;
#ifndef _I2C_TEGRA_DISABLE_CAR_ACCESS
	const struct tegra_clk *slow_clk;
	const struct tegra_clk *div_clk;
	const struct tegra_rst *reset;
#endif
	uint32_t irq;
};

struct tegra_i2c_ctlr
{
	/*
	 * The driver relies on this being the first field, since it casts
	 * pointers between structs tegra_i2c_id and tegra_i2c_ctlr.
	 */
	struct tegra_i2c_id conf;
	QueueHandle_t irq_queue;
	uint32_t clk_divisor;
	struct tegra_i2c_xfer *xfers;
	uint32_t num_xfers;
	/* tx, rx refer to which FIFO not I2C transfer direction */
	uint32_t cur_tx_xfer_idx;
	bool cur_tx_xfer_sent_header;
	bool broken;
	uint8_t *cur_tx_xfer_buf;
	uint32_t cur_tx_xfer_count_remaining;
	uint32_t cur_rx_xfer_idx;
	uint8_t *cur_rx_xfer_buf;
	uint32_t cur_rx_xfer_count_remaining;
};

#endif
