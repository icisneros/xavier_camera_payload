/*
 * Copyright (c) 2014-2015, NVIDIA CORPORATION.  All rights reserved.
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
#include <uart-tegra-hw-params.h>
#include <uart-tegra-priv.h>

#ifdef TEGRA_UARTC_ENABLED
struct tegra_uart_ctlr tegra_uart_ctlr_uartc = {
	.id = {
		.devname = "uartc",
		.base_addr = NV_ADDRESS_MAP_UARTC_BASE,
		.clk = tegra_clk_uartc,
		.reset = tegra_rst_uartc,
		.irq = TEGRA_UARTC_IRQ,
	},
};
extern struct tegra_uart_id tegra_uart_id_uartc __attribute__((alias("tegra_uart_ctlr_uartc")));
#endif

#ifdef TEGRA_UARTG_ENABLED
struct tegra_uart_ctlr tegra_uart_ctlr_uartg = {
	.id = {
		.devname = "uartg",
		.base_addr = NV_ADDRESS_MAP_UARTG_BASE,
		.clk = tegra_clk_uartg,
		.reset = tegra_rst_uartg,
		.irq = TEGRA_UARTG_IRQ,
	},
};
extern struct tegra_uart_id tegra_uart_id_uartg __attribute__((alias("tegra_uart_ctlr_uartg")));
#endif
