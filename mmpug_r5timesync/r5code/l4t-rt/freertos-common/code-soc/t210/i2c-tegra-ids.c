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

#include <clk-tegra-hw.h>
#include <i2c-tegra-priv.h>
#include <irqs-hw.h>

struct tegra_i2c_ctlr tegra_i2c_ctlr_vi_i2c = {
	.conf = {
		.devname = "vii2c",
		.base_addr = 0x546c0c00,
		.offset_mult = 4,
#ifndef _I2C_TEGRA_DISABLE_CAR_ACCESS
		.slow_clk = tegra_clk_i2c_slow,
		.div_clk = tegra_clk_vi_i2c,
		.reset = tegra_rst_vi_i2c,
#endif
		.irq = TEGRA_IRQ_VI_I2C,
	},
};
extern struct tegra_i2c_id tegra_i2c_id_vi_i2c __attribute__((alias("tegra_i2c_ctlr_vi_i2c")));

struct tegra_i2c_ctlr tegra_i2c_ctlr_i2c1 = {
	.conf = {
		.devname = "i2c1",
		.base_addr = 0x7000c000,
		.offset_mult = 1,
#ifndef _I2C_TEGRA_DISABLE_CAR_ACCESS
		.slow_clk = tegra_clk_i2c_slow,
		.div_clk = tegra_clk_i2c1,
		.reset = tegra_rst_i2c1,
#endif
		.irq = TEGRA_IRQ_I2C1,
	},
};
extern struct tegra_i2c_id tegra_i2c_id_i2c1 __attribute__((alias("tegra_i2c_ctlr_i2c1")));
