/*
 * Copyright (c) 2015-2017, NVIDIA CORPORATION. All rights reserved.
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

#ifndef _CLK_TEGRA_HW_H_
#define _CLK_TEGRA_HW_H_

extern const struct tegra_clk tegra_clk_uarta_data;
extern const struct tegra_clk tegra_clk_uartc_data;
extern const struct tegra_clk tegra_clk_uartf_data;
extern const struct tegra_clk tegra_clk_uartg_data;
extern const struct tegra_clk tegra_clk_uarth_data;
extern const struct tegra_clk tegra_clk_aon_i2c_slow_data;
extern const struct tegra_clk tegra_clk_i2c_slow_data;
extern const struct tegra_clk tegra_clk_i2c1_data;
extern const struct tegra_clk tegra_clk_i2c2_data;
extern const struct tegra_clk tegra_clk_i2c3_data;
extern const struct tegra_clk tegra_clk_i2c8_data;
extern const struct tegra_clk tegra_clk_i2c10_data;
extern const struct tegra_clk tegra_clk_aodmic_data;
extern const struct tegra_clk tegra_clk_spi2_data;

extern struct tegra_parent_clk tegra_parent_clk_pll_aon_data;
extern struct tegra_parent_clk tegra_parent_clk_pll_p_data;
extern struct tegra_parent_clk tegra_parent_clk_osc_undiv_data;

extern const struct tegra_rst tegra_rst_uarta_data;
extern const struct tegra_rst tegra_rst_uartc_data;
extern const struct tegra_rst tegra_rst_uartf_data;
extern const struct tegra_rst tegra_rst_uartg_data;
extern const struct tegra_rst tegra_rst_uarth_data;
extern const struct tegra_rst tegra_rst_i2c1_data;
extern const struct tegra_rst tegra_rst_i2c2_data;
extern const struct tegra_rst tegra_rst_i2c3_data;
extern const struct tegra_rst tegra_rst_i2c8_data;
extern const struct tegra_rst tegra_rst_i2c10_data;
extern const struct tegra_rst tegra_rst_aodmic_data;
extern const struct tegra_rst tegra_rst_aon_gpcdma_data;
extern const struct tegra_rst tegra_rst_sce_gpcdma_data;
extern const struct tegra_rst tegra_rst_rce_gpcdma_data;
extern const struct tegra_rst tegra_rst_spi2_data;
extern const struct tegra_rst tegra_rst_can0_data;
extern const struct tegra_rst tegra_rst_can1_data;

#define tegra_clk_uarta (&tegra_clk_uarta_data)
#define tegra_clk_uartc (&tegra_clk_uartc_data)
#define tegra_clk_uartf (&tegra_clk_uartf_data)
#define tegra_clk_uartg (&tegra_clk_uartg_data)
#define tegra_clk_uarth (&tegra_clk_uarth_data)
#define tegra_clk_aon_i2c_slow (&tegra_clk_aon_i2c_slow_data)
#define tegra_clk_i2c_slow (&tegra_clk_i2c_slow_data)
#define tegra_clk_i2c1 (&tegra_clk_i2c1_data)
#define tegra_clk_i2c2 (&tegra_clk_i2c2_data)
#define tegra_clk_i2c3 (&tegra_clk_i2c3_data)
#define tegra_clk_i2c8 (&tegra_clk_i2c8_data)
#define tegra_clk_i2c10 (&tegra_clk_i2c10_data)
#define tegra_clk_aodmic (&tegra_clk_aodmic_data)
#define tegra_clk_spi2 (&tegra_clk_spi2_data)
#define tegra_parent_clk_pll_aon tegra_parent_clk_pll_aon_data
#define tegra_parent_clk_pll_p tegra_parent_clk_pll_p_data
#define tegra_parent_clk_osc_undiv tegra_parent_clk_osc_undiv_data

#define tegra_rst_uarta (&tegra_rst_uarta_data)
#define tegra_rst_uartc (&tegra_rst_uartc_data)
#define tegra_rst_uartf (&tegra_rst_uartf_data)
#define tegra_rst_uartg (&tegra_rst_uartg_data)
#define tegra_rst_uarth (&tegra_rst_uarth_data)
#define tegra_rst_i2c1 (&tegra_rst_i2c1_data)
#define tegra_rst_i2c2 (&tegra_rst_i2c2_data)
#define tegra_rst_i2c3 (&tegra_rst_i2c3_data)
#define tegra_rst_i2c8 (&tegra_rst_i2c8_data)
#define tegra_rst_i2c10 (&tegra_rst_i2c10_data)
#define tegra_rst_aodmic (&tegra_rst_aodmic_data)
#define tegra_rst_aon_gpcdma (&tegra_rst_aon_gpcdma_data)
#define tegra_rst_sce_gpcdma (&tegra_rst_sce_gpcdma_data)
#define tegra_rst_rce_gpcdma (&tegra_rst_rce_gpcdma_data)
#define tegra_rst_spi2 (&tegra_rst_spi2_data)
#define tegra_rst_can0 (&tegra_rst_can0_data)
#define tegra_rst_can1 (&tegra_rst_can1_data)

#endif
