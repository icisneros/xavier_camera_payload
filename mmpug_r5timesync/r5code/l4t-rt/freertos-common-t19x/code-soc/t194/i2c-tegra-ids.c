/*
 * Copyright (c) 2016, NVIDIA CORPORATION. All rights reserved.
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

#include <address_map_new.h>

#include <clk-tegra-hw.h>
#include <i2c-tegra-hw-params.h>
#include <i2c-tegra-priv.h>
#include <irqs-hw.h>

#ifdef TEGRA_I2C1_ENABLED
struct tegra_i2c_ctlr tegra_i2c_ctlr_i2c1 = {
	.conf = {
		.devname = "i2c1",
		.base_addr = NV_ADDRESS_MAP_I2C1_BASE,
		.offset_mult = 1,
		.irq = TEGRA_I2C1_IRQ,
#ifndef _I2C_TEGRA_DISABLE_CAR_ACCESS
		.slow_clk = tegra_clk_i2c_slow,
		.div_clk = tegra_clk_i2c1,
		.reset = tegra_rst_i2c1,
#endif
	},
};
extern struct tegra_i2c_id tegra_i2c_id_i2c1 __attribute__((alias("tegra_i2c_ctlr_i2c1")));
#endif

#ifdef TEGRA_I2C2_ENABLED
struct tegra_i2c_ctlr tegra_i2c_ctlr_i2c2 = {
	.conf = {
		.devname = "i2c2",
		.base_addr = NV_ADDRESS_MAP_I2C2_BASE,
		.offset_mult = 1,
		.irq = TEGRA_I2C2_IRQ,
#ifndef _I2C_TEGRA_DISABLE_CAR_ACCESS
		.slow_clk = tegra_clk_aon_i2c_slow,
		.div_clk = tegra_clk_i2c2,
		.reset = tegra_rst_i2c2,
#endif
	},
};
extern struct tegra_i2c_id tegra_i2c_id_i2c2 __attribute__((alias("tegra_i2c_ctlr_i2c2")));
#endif

#ifdef TEGRA_I2C3_ENABLED
struct tegra_i2c_ctlr tegra_i2c_ctlr_i2c3 = {
	.conf = {
		.devname = "i2c3",
		.base_addr = NV_ADDRESS_MAP_I2C3_BASE,
		.offset_mult = 1,
		.irq = TEGRA_I2C3_IRQ,
#ifndef _I2C_TEGRA_DISABLE_CAR_ACCESS
		.slow_clk = tegra_clk_i2c_slow,
		.div_clk = tegra_clk_i2c3,
		.reset = tegra_rst_i2c3,
#endif
	},
};
extern struct tegra_i2c_id tegra_i2c_id_i2c3 __attribute__((alias("tegra_i2c_ctlr_i2c3")));
#endif

#ifdef TEGRA_I2C8_ENABLED
struct tegra_i2c_ctlr tegra_i2c_ctlr_i2c8 = {
	.conf = {
		.devname = "i2c8",
		.base_addr = NV_ADDRESS_MAP_I2C8_BASE,
		.offset_mult = 1,
		.irq = TEGRA_I2C8_IRQ,
#ifndef _I2C_TEGRA_DISABLE_CAR_ACCESS
		.slow_clk = tegra_clk_aon_i2c_slow,
		.div_clk = tegra_clk_i2c8,
		.reset = tegra_rst_i2c8,
#endif
	},
};
extern struct tegra_i2c_id tegra_i2c_id_i2c8 __attribute__((alias("tegra_i2c_ctlr_i2c8")));
#endif

#ifdef TEGRA_I2C10_ENABLED
struct tegra_i2c_ctlr tegra_i2c_ctlr_i2c10 = {
	.conf = {
		.devname = "i2c10",
		.base_addr = NV_ADDRESS_MAP_I2C10_BASE,
		.offset_mult = 1,
		.irq = TEGRA_I2C10_IRQ,
#ifndef _I2C_TEGRA_DISABLE_CAR_ACCESS
		.slow_clk = tegra_clk_aon_i2c_slow,
		.div_clk = tegra_clk_i2c10,
		.reset = tegra_rst_i2c10,
#endif
	},
};
extern struct tegra_i2c_id tegra_i2c_id_i2c10 __attribute__((alias("tegra_i2c_ctlr_i2c10")));
#endif
