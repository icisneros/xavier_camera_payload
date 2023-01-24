/*
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
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
#include <spi-tegra-hw-params.h>
#include <spi-tegra-priv.h>

#ifdef TEGRA_SPI2_ENABLED
struct tegra_spi_ctlr tegra_spi_data_spi2 = {
	.conf = {
		.devname = "spi2",
		.base_addr = NV_ADDRESS_MAP_SPI2_BASE,
		.div_clk = tegra_clk_spi2,
		.reset = tegra_rst_spi2,
		.irq = TEGRA_SPI2_IRQ,
	},
};
extern struct tegra_spi_id tegra_spi_id_spi2 __attribute__((alias("tegra_spi_data_spi2")));
#endif

#ifdef TEGRA_SPI4_ENABLED
struct tegra_spi_ctlr tegra_spi_data_spi4 = {
	.conf = {
		.devname = "spi4",
		.base_addr = NV_ADDRESS_MAP_SPI4_BASE,
		.div_clk = tegra_clk_spi4,
		.reset = tegra_rst_spi4,
		.irq = TEGRA_SPI4_IRQ,
	},
};
extern struct tegra_spi_id tegra_spi_id_spi4 __attribute__((alias("tegra_spi_data_spi4")));
#endif
