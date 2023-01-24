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

#ifndef _DEBUG_HW_H_
#define _DEBUG_HW_H_

#include <address_map_new.h>

#ifdef _NV_BUILD_FPGA_
#define TEGRA_DBG_UART_BASE	NV_ADDRESS_MAP_SCE_FPGA_UART_BASE
#define TEGRA_DBG_UART_CLK	tegra_clk_uartsce
#define TEGRA_DBG_UART_BAUD	115200
#else
#ifndef TEGRA_DBG_HW_SEL_UARTG
#define TEGRA_DBG_UART_BASE	NV_ADDRESS_MAP_UARTA_BASE
#define TEGRA_DBG_UART_CLK	tegra_clk_uarta
#define TEGRA_DBG_UART_RST	tegra_rst_uarta
#define TEGRA_DBG_UART_BAUD	115200
#else
#define TEGRA_DBG_UART_BASE	NV_ADDRESS_MAP_UARTG_BASE
#endif
#endif

#endif