/*
 * Copyright (c) 2015-2017, NVIDIA CORPORATION.  All rights reserved.
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

#include <stdint.h>

#include <address_map_new.h>
#include <arclk_rst.h>
#include <nvrm_drf.h>

#include <clk-tegra.h>
#include <clk-tegra-hw.h>
#ifdef _NV_BUILD_FPGA_
#include <clk-tegra-hw-cpu.h>
#endif
#include <delay.h>
#include <macros.h>
#include <barriers.h>
#include <reg-access.h>

#define CLK_RST_CONTROLLER_CLK_OUT_ENB_0		0x00
#define CLK_RST_CONTROLLER_CLK_OUT_ENB_SET_0		0x04
#define CLK_RST_CONTROLLER_CLK_OUT_ENB_CLR_0		0x08

#define CLK_RST_CONTROLLER_RST_DEV_0			0x00
#define CLK_RST_CONTROLLER_RST_DEV_SET_0		0x04
#define CLK_RST_CONTROLLER_RST_DEV_CLR_0		0x08

#define LOWBIT     0
#define HIGHBIT    1
#define FIELD_RANGE(x) { \
	[LOWBIT]  = NV_FIELD_LOWBIT(x), \
	[HIGHBIT] = NV_FIELD_HIGHBIT(x) \
}

struct tegra_clk {
	uint32_t enb_base_reg;
	uint32_t src_reg;
	uint8_t div_range[2]; /* [0]=LOWBIT [1]=HIGHBIT */
	uint32_t div_enb; /* 0 if enable bit is not available */
	uint8_t frac_width;
	uint32_t clk_src;
	const struct tegra_parent_clk *parent;
};

struct tegra_parent_clk {
	uint32_t rate;
};

struct tegra_rst {
	uint32_t rst_base_reg;
};

struct tegra_parent_clk tegra_parent_clk_clk_s_data = {
	.rate = 32768,
};

struct tegra_parent_clk tegra_parent_clk_clk_m_data = {
	.rate = 0,
};

struct tegra_parent_clk tegra_parent_clk_pll_p_data = {
	.rate = 408000000,
};

struct tegra_parent_clk tegra_parent_clk_pll_aon_data = {
	.rate = 0,
};

const struct tegra_clk tegra_clk_uartc_data = {
	.enb_base_reg = CLK_RST_CONTROLLER_CLK_OUT_ENB_UARTC_0,
	.src_reg = CLK_RST_CONTROLLER_CLK_SOURCE_UARTC_0,
	.div_range = FIELD_RANGE(CLK_RST_CONTROLLER_CLK_SOURCE_UARTC_0_UARTC_CLK_DIVISOR_RANGE),
#if defined(_NV_BUILD_FPGA_) || defined(_NV_BUILD_LINSIM_)
	.div_enb = NV_DRF_DEF(CLK_RST_CONTROLLER, CLK_SOURCE_UARTC, UARTC_DIV_ENB, DISABLE),
#else
	.div_enb = NV_DRF_DEF(CLK_RST_CONTROLLER, CLK_SOURCE_UARTC, UARTC_DIV_ENB, ENABLE),
#endif
	.frac_width = 1,
	.clk_src = NV_DRF_DEF(CLK_RST_CONTROLLER, CLK_SOURCE_UARTC, UARTC_CLK_SRC, CLK_M),
	.parent = &tegra_parent_clk_clk_m_data,
};

const struct tegra_rst tegra_rst_uartc_data = {
	.rst_base_reg = CLK_RST_CONTROLLER_RST_DEV_UARTC_0,
};

const struct tegra_clk tegra_clk_uartg_data = {
	.enb_base_reg = CLK_RST_CONTROLLER_CLK_OUT_ENB_UARTG_0,
	.src_reg = CLK_RST_CONTROLLER_CLK_SOURCE_UARTG_0,
	.div_range = FIELD_RANGE(CLK_RST_CONTROLLER_CLK_SOURCE_UARTG_0_UARTG_CLK_DIVISOR_RANGE),
#if defined(_NV_BUILD_FPGA_) || defined(_NV_BUILD_LINSIM_)
	.div_enb = NV_DRF_DEF(CLK_RST_CONTROLLER, CLK_SOURCE_UARTG, UARTG_DIV_ENB, DISABLE),
#else
	.div_enb = NV_DRF_DEF(CLK_RST_CONTROLLER, CLK_SOURCE_UARTG, UARTG_DIV_ENB, ENABLE),
#endif
	.frac_width = 1,
	.clk_src = NV_DRF_DEF(CLK_RST_CONTROLLER, CLK_SOURCE_UARTG, UARTG_CLK_SRC, CLK_M),
	.parent = &tegra_parent_clk_clk_m_data,
};

const struct tegra_rst tegra_rst_uartg_data = {
	.rst_base_reg = CLK_RST_CONTROLLER_RST_DEV_UARTG_0,
};

const struct tegra_clk tegra_clk_uarta_data = {
	.enb_base_reg = CLK_RST_CONTROLLER_CLK_OUT_ENB_UARTA_0,
	.src_reg = CLK_RST_CONTROLLER_CLK_SOURCE_UARTA_0,
	.div_range = FIELD_RANGE(CLK_RST_CONTROLLER_CLK_SOURCE_UARTA_0_UARTA_CLK_DIVISOR_RANGE),
#if defined(_NV_BUILD_FPGA_) || defined(_NV_BUILD_LINSIM_)
	.div_enb = NV_DRF_DEF(CLK_RST_CONTROLLER, CLK_SOURCE_UARTA, UARTA_DIV_ENB, DISABLE),
#else
	.div_enb = NV_DRF_DEF(CLK_RST_CONTROLLER, CLK_SOURCE_UARTA, UARTA_DIV_ENB, ENABLE),
#endif
	.frac_width = 1,
	.clk_src = NV_DRF_DEF(CLK_RST_CONTROLLER, CLK_SOURCE_UARTA, UARTA_CLK_SRC, PLLP_OUT0),
	.parent = &tegra_parent_clk_pll_p_data,
};

const struct tegra_rst tegra_rst_uarta_data = {
	.rst_base_reg = CLK_RST_CONTROLLER_RST_DEV_UARTA_0,
};

const struct tegra_clk tegra_clk_aon_i2c_slow_data = {
	.enb_base_reg = CLK_RST_CONTROLLER_CLK_OUT_ENB_AON_I2C_SLOW_0,
	.src_reg = CLK_RST_CONTROLLER_CLK_SOURCE_AON_I2C_SLOW_0,
	.div_range = FIELD_RANGE(CLK_RST_CONTROLLER_CLK_SOURCE_AON_I2C_SLOW_0_AON_I2C_SLOW_CLK_DIVISOR_RANGE),
	.div_enb = 0,
	.frac_width = 1,
	.clk_src = NV_DRF_DEF(CLK_RST_CONTROLLER, CLK_SOURCE_AON_I2C_SLOW, AON_I2C_SLOW_CLK_SRC, CLK_S),
	.parent = &tegra_parent_clk_clk_s_data,
};

const struct tegra_clk tegra_clk_i2c_slow_data = {
	.enb_base_reg = CLK_RST_CONTROLLER_CLK_OUT_ENB_I2C_SLOW_0,
	.src_reg = CLK_RST_CONTROLLER_CLK_SOURCE_I2C_SLOW_0,
	.div_range = FIELD_RANGE(CLK_RST_CONTROLLER_CLK_SOURCE_I2C_SLOW_0_I2C_SLOW_CLK_DIVISOR_RANGE),
	.div_enb = 0,
	.frac_width = 1,
	.clk_src = NV_DRF_DEF(CLK_RST_CONTROLLER, CLK_SOURCE_I2C_SLOW, I2C_SLOW_CLK_SRC, CLK_S),
	.parent = &tegra_parent_clk_clk_s_data,
};

const struct tegra_clk tegra_clk_i2c1_data = {
	.enb_base_reg = CLK_RST_CONTROLLER_CLK_OUT_ENB_I2C1_0,
	.src_reg = CLK_RST_CONTROLLER_CLK_SOURCE_I2C1_0,
	.div_range = FIELD_RANGE(CLK_RST_CONTROLLER_CLK_SOURCE_I2C1_0_I2C1_CLK_DIVISOR_RANGE),
	.div_enb = 0,
	.frac_width = 0,
	.clk_src = NV_DRF_DEF(CLK_RST_CONTROLLER, CLK_SOURCE_I2C1, I2C1_CLK_SRC, PLLP_OUT0),
	.parent = &tegra_parent_clk_pll_p_data,
};

const struct tegra_rst tegra_rst_i2c1_data = {
	.rst_base_reg = CLK_RST_CONTROLLER_RST_DEV_I2C1_0,
};

const struct tegra_clk tegra_clk_i2c2_data = {
	.enb_base_reg = CLK_RST_CONTROLLER_CLK_OUT_ENB_I2C2_0,
	.src_reg = CLK_RST_CONTROLLER_CLK_SOURCE_I2C2_0,
	.div_range = FIELD_RANGE(CLK_RST_CONTROLLER_CLK_SOURCE_I2C2_0_I2C2_CLK_DIVISOR_RANGE),
	.div_enb = 0,
	.frac_width = 0,
	.clk_src = NV_DRF_DEF(CLK_RST_CONTROLLER, CLK_SOURCE_I2C2, I2C2_CLK_SRC, PLLP_OUT0),
	.parent = &tegra_parent_clk_pll_p_data,
};

const struct tegra_rst tegra_rst_i2c2_data = {
	.rst_base_reg = CLK_RST_CONTROLLER_RST_DEV_I2C2_0,
};

const struct tegra_clk tegra_clk_i2c3_data = {
	.enb_base_reg = CLK_RST_CONTROLLER_CLK_OUT_ENB_I2C3_0,
	.src_reg = CLK_RST_CONTROLLER_CLK_SOURCE_I2C3_0,
	.div_range = FIELD_RANGE(CLK_RST_CONTROLLER_CLK_SOURCE_I2C3_0_I2C3_CLK_DIVISOR_RANGE),
	.div_enb = 0,
	.frac_width = 0,
	.clk_src = NV_DRF_DEF(CLK_RST_CONTROLLER, CLK_SOURCE_I2C3, I2C3_CLK_SRC, PLLP_OUT0),
	.parent = &tegra_parent_clk_pll_p_data,
};

const struct tegra_rst tegra_rst_i2c3_data = {
	.rst_base_reg = CLK_RST_CONTROLLER_RST_DEV_I2C3_0,
};

const struct tegra_clk tegra_clk_i2c8_data = {
	.enb_base_reg = CLK_RST_CONTROLLER_CLK_OUT_ENB_I2C8_0,
	.src_reg = CLK_RST_CONTROLLER_CLK_SOURCE_I2C8_0,
	.div_range = FIELD_RANGE(CLK_RST_CONTROLLER_CLK_SOURCE_I2C8_0_I2C8_CLK_DIVISOR_RANGE),
	.div_enb = 0,
	.frac_width = 0,
	.clk_src = NV_DRF_DEF(CLK_RST_CONTROLLER, CLK_SOURCE_I2C8, I2C8_CLK_SRC, PLLP_OUT0),
	.parent = &tegra_parent_clk_pll_p_data,
};

const struct tegra_rst tegra_rst_i2c8_data = {
	.rst_base_reg = CLK_RST_CONTROLLER_RST_DEV_I2C8_0,
};

const struct tegra_clk tegra_clk_i2c10_data = {
	.enb_base_reg = CLK_RST_CONTROLLER_CLK_OUT_ENB_I2C10_0,
	.src_reg = CLK_RST_CONTROLLER_CLK_SOURCE_I2C10_0,
	.div_range = FIELD_RANGE(CLK_RST_CONTROLLER_CLK_SOURCE_I2C10_0_I2C10_CLK_DIVISOR_RANGE),
	.div_enb = 0,
	.frac_width = 0,
	.clk_src = NV_DRF_DEF(CLK_RST_CONTROLLER, CLK_SOURCE_I2C10, I2C10_CLK_SRC, PLLP_OUT0),
	.parent = &tegra_parent_clk_pll_p_data,
};

const struct tegra_rst tegra_rst_i2c10_data = {
	.rst_base_reg = CLK_RST_CONTROLLER_RST_DEV_I2C10_0,
};

const struct tegra_rst tegra_rst_aon_gpcdma_data = {
	.rst_base_reg = CLK_RST_CONTROLLER_RST_DEV_AON_DMA_0,
};

const struct tegra_rst tegra_rst_sce_gpcdma_data = {
	.rst_base_reg = CLK_RST_CONTROLLER_RST_DEV_SCE_DMA_0,
};

const struct tegra_clk tegra_clk_aodmic_data = {
	.enb_base_reg = CLK_RST_CONTROLLER_CLK_OUT_ENB_DMIC5_0,
	.src_reg = CLK_RST_CONTROLLER_CLK_SOURCE_DMIC5_0,
	.div_range = FIELD_RANGE(CLK_RST_CONTROLLER_CLK_SOURCE_DMIC5_0_DMIC5_CLK_DIVISOR_RANGE),
	.div_enb = 0,
	.frac_width = 1,
	.clk_src = NV_DRF_DEF(CLK_RST_CONTROLLER,CLK_SOURCE_DMIC5, DMIC5_CLK_SRC, CLK_M),
	.parent = &tegra_parent_clk_clk_m_data,
};

const struct tegra_rst tegra_rst_aodmic_data = {
	.rst_base_reg = CLK_RST_CONTROLLER_RST_DEV_DMIC5_0,
};

const struct tegra_clk tegra_clk_spi2_data = {
	.enb_base_reg = CLK_RST_CONTROLLER_CLK_OUT_ENB_SPI2_0,
	.src_reg = CLK_RST_CONTROLLER_CLK_SOURCE_SPI2_0,
	.div_range = FIELD_RANGE(CLK_RST_CONTROLLER_CLK_SOURCE_SPI2_0_SPI2_CLK_DIVISOR_RANGE),
	.div_enb = 0,
	.frac_width = 1,
	.clk_src = NV_DRF_DEF(CLK_RST_CONTROLLER, CLK_SOURCE_SPI2, SPI2_CLK_SRC, PLLP_OUT0),
	.parent = &tegra_parent_clk_pll_p_data,
};

const struct tegra_clk tegra_clk_spi4_data = {
	.enb_base_reg = CLK_RST_CONTROLLER_CLK_OUT_ENB_SPI4_0,
	.src_reg = CLK_RST_CONTROLLER_CLK_SOURCE_SPI4_0,
	.div_range = FIELD_RANGE(CLK_RST_CONTROLLER_CLK_SOURCE_SPI4_0_SPI4_CLK_DIVISOR_RANGE),
	.div_enb = 0,
	.frac_width = 1,
	.clk_src = NV_DRF_DEF(CLK_RST_CONTROLLER, CLK_SOURCE_SPI4, SPI4_CLK_SRC, PLLP_OUT0),
	.parent = &tegra_parent_clk_pll_p_data,
};

const struct tegra_rst tegra_rst_spi2_data = {
	.rst_base_reg = CLK_RST_CONTROLLER_RST_DEV_SPI2_0,
};

const struct tegra_rst tegra_rst_spi4_data = {
	.rst_base_reg = CLK_RST_CONTROLLER_RST_DEV_SPI4_0,
};

const struct tegra_rst tegra_rst_can0_data = {
	.rst_base_reg = CLK_RST_CONTROLLER_RST_DEV_CAN1_0,
};

const struct tegra_rst tegra_rst_can1_data = {
	.rst_base_reg = CLK_RST_CONTROLLER_RST_DEV_CAN2_0,
};

#ifdef _NV_BUILD_FPGA_
const struct tegra_clk tegra_clk_uartsce_data = {
	/* FIXME: There actually isn't an enable register */
	.enb_base_reg = 0,
	.src_reg = (NV_ADDRESS_MAP_SCE_FPGA_MISC_BASE + 0x5000) - NV_ADDRESS_MAP_CAR_BASE,
	/* FIXME: validate this */
	.div_range = FIELD_RANGE(15:0),
	.div_enb = NV_FIELD_BITS(1, 24:24),
	.frac_width = 0,
	.clk_src = 0,
	.parent = &tegra_parent_clk_clk_m_data,
};
#endif

int tegra_clk_init(void)
{
	uint32_t clk_m_rate;
#ifndef _NV_BUILD_FPGA_
	uint32_t val;
	uint32_t osc_freq;

	val = readl(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_OSC_CTRL_0);

	osc_freq = NV_DRF_VAL(CLK_RST_CONTROLLER, OSC_CTRL, OSC_FREQ, val);
	switch (osc_freq) {
	case 0:
		clk_m_rate = 13000000;
		break;
	case 1:
		clk_m_rate = 16800000;
		break;
	case 4:
		clk_m_rate = 19200000;
		break;
	case 5:
		clk_m_rate = 38400000;
		break;
	case 8:
		clk_m_rate = 12000000;
		break;
	case 9:
		clk_m_rate = 48000000;
		break;
	case 12:
		clk_m_rate = 26000000;
		break;
	default:
		return 1;
	}
#else
	/* FPGA */
	clk_m_rate = FPGA_FREQUENCY_CLK_M;
#endif
	tegra_parent_clk_clk_m_data.rate = clk_m_rate;

	return 0;
}

int tegra_clk_enable(const struct tegra_clk *clk)
{
	writel(BIT(0), NV_ADDRESS_MAP_CAR_BASE + clk->enb_base_reg + CLK_RST_CONTROLLER_CLK_OUT_ENB_SET_0);
	barrier_memory_complete();

	return 0;
}

int tegra_clk_disable(const struct tegra_clk *clk)
{
	barrier_memory_complete();
	writel(BIT(0), NV_ADDRESS_MAP_CAR_BASE + clk->enb_base_reg + CLK_RST_CONTROLLER_CLK_OUT_ENB_CLR_0);
	barrier_memory_complete();

	return 0;
}

int tegra_clk_reset_set(const struct tegra_rst *rst)
{
	barrier_memory_complete();
	writel(BIT(0), NV_ADDRESS_MAP_CAR_BASE + rst->rst_base_reg + CLK_RST_CONTROLLER_RST_DEV_SET_0);
	barrier_memory_complete();

	return 0;
}

int tegra_clk_reset_clear(const struct tegra_rst *rst)
{
	writel(BIT(0), NV_ADDRESS_MAP_CAR_BASE + rst->rst_base_reg + CLK_RST_CONTROLLER_RST_DEV_CLR_0);
	barrier_memory_complete();

	return 0;
}

int tegra_clk_reset_pulse(const struct tegra_rst *rst, uint32_t delay_us)
{
	int ret;

	ret = tegra_clk_reset_set(rst);
	if (ret)
		return ret;
	udelay(delay_us);
	return tegra_clk_reset_clear(rst);
}

int tegra_clk_set_rate(const struct tegra_clk *clk, uint32_t rate_hz)
{
	uint32_t divisor;
	uint32_t val;

	if (!clk->parent->rate)
		return 1;
	/*
	 * For now, we always round to ensure the clock runs no faster than
	 * the requested rate. If some clocks need other rounding options,
	 * we can add flags to struct tegra_clk for that.
	 */
	divisor = ((clk->parent->rate << clk->frac_width) + rate_hz - 1) / rate_hz;
	divisor -= (1 << clk->frac_width);
	if (divisor & ~NV_FIELD_MASK(clk->div_range[HIGHBIT]:clk->div_range[LOWBIT]))
		return 1;

	val = NV_FIELD_BITS(divisor, clk->div_range[HIGHBIT]:clk->div_range[LOWBIT]) |
	      clk->clk_src | clk->div_enb;

	barrier_memory_complete();
	writel(val, NV_ADDRESS_MAP_CAR_BASE + clk->src_reg);
	barrier_memory_complete();

	return 0;
}

int tegra_parent_clk_set_rate(struct tegra_parent_clk *clk, uint32_t rate_hz)
{
	clk->rate = rate_hz;
	return 0;
}
