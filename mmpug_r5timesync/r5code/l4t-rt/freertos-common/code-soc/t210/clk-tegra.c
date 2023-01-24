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

#include <stdint.h>

#include <nvrm_drf.h>

#include <clk-tegra.h>
#include <delay.h>
#include <macros.h>
#include <reg-access.h>

#define CLK_RST_CONTROLLER_BASE				0x60006000

#define CLK_RST_CONTROLLER_OSC_CTRL_0			0x050
#define CLK_RST_CONTROLLER_CLK_SOURCE_I2C1_0		0x124
#define CLK_RST_CONTROLLER_CLK_ENB_X_SET_0		0x284
#define CLK_RST_CONTROLLER_CLK_ENB_X_CLR_0		0x288
#define CLK_RST_CONTROLLER_RST_DEVX_SET_0		0x290
#define CLK_RST_CONTROLLER_RST_DEVX_CLR_0		0x294
#define CLK_RST_CONTROLLER_CLK_ENB_Y_SET_0		0x29c
#define CLK_RST_CONTROLLER_CLK_ENB_Y_CLR_0		0x2a0
#define CLK_RST_CONTROLLER_RST_DEVY_SET_0		0x2a8
#define CLK_RST_CONTROLLER_RST_DEVY_CLR_0		0x2ac
#define CLK_RST_CONTROLLER_RST_DEVL_SET_0		0x300
#define CLK_RST_CONTROLLER_RST_DEVL_CLR_0		0x304
#define CLK_RST_CONTROLLER_RST_DEVH_SET_0		0x308
#define CLK_RST_CONTROLLER_RST_DEVH_CLR_0		0x30c
#define CLK_RST_CONTROLLER_RST_DEVU_SET_0		0x310
#define CLK_RST_CONTROLLER_RST_DEVU_CLR_0		0x314
#define CLK_RST_CONTROLLER_CLK_ENB_L_SET_0		0x320
#define CLK_RST_CONTROLLER_CLK_ENB_L_CLR_0		0x324
#define CLK_RST_CONTROLLER_CLK_ENB_H_SET_0		0x328
#define CLK_RST_CONTROLLER_CLK_ENB_H_CLR_0		0x32c
#define CLK_RST_CONTROLLER_CLK_ENB_U_SET_0		0x330
#define CLK_RST_CONTROLLER_CLK_ENB_U_CLR_0		0x334
#define CLK_RST_CONTROLLER_CLK_SOURCE_I2C_SLOW_0	0x3fc
#define CLK_RST_CONTROLLER_RST_DEVV_SET_0		0x430
#define CLK_RST_CONTROLLER_RST_DEVV_CLR_0		0x434
#define CLK_RST_CONTROLLER_RST_DEVW_SET_0		0x438
#define CLK_RST_CONTROLLER_RST_DEVW_CLR_0		0x43c
#define CLK_RST_CONTROLLER_CLK_ENB_V_SET_0		0x440
#define CLK_RST_CONTROLLER_CLK_ENB_V_CLR_0		0x444
#define CLK_RST_CONTROLLER_CLK_ENB_W_SET_0		0x448
#define CLK_RST_CONTROLLER_CLK_ENB_W_CLR_0		0x44c
#define CLK_RST_CONTROLLER_CLK_SOURCE_VI_I2C_0		0x6c8

#define CLK_RST_CONTROLLER_OSC_CTRL_0_OSC_FREQ_RANGE	31:28

#define CLK_RST_CONTROLLER_CLK_SOURCE_0_DIVISOR_RANGE	15:0
#define CLK_RST_CONTROLLER_CLK_SOURCE_0_CLK_SRC_RANGE	31:29

struct tegra_clk {
	uint16_t rst_enb_id;
	uint16_t reg;
	uint8_t div_bits;
	uint8_t frac_width;
	uint8_t clk_src_val;
	const uint32_t *parent_rate_ptr;
};

static const uint32_t clk_enb_set_regs[] = {
	CLK_RST_CONTROLLER_CLK_ENB_L_SET_0,
	CLK_RST_CONTROLLER_CLK_ENB_H_SET_0,
	CLK_RST_CONTROLLER_CLK_ENB_U_SET_0,
	CLK_RST_CONTROLLER_CLK_ENB_V_SET_0,
	CLK_RST_CONTROLLER_CLK_ENB_W_SET_0,
	CLK_RST_CONTROLLER_CLK_ENB_X_SET_0,
	CLK_RST_CONTROLLER_CLK_ENB_Y_SET_0,
};

static const uint32_t clk_enb_clr_regs[] = {
	CLK_RST_CONTROLLER_CLK_ENB_L_CLR_0,
	CLK_RST_CONTROLLER_CLK_ENB_H_CLR_0,
	CLK_RST_CONTROLLER_CLK_ENB_U_CLR_0,
	CLK_RST_CONTROLLER_CLK_ENB_V_CLR_0,
	CLK_RST_CONTROLLER_CLK_ENB_W_CLR_0,
	CLK_RST_CONTROLLER_CLK_ENB_X_CLR_0,
	CLK_RST_CONTROLLER_CLK_ENB_Y_CLR_0,
};

static const uint32_t rst_dev_set_regs[] = {
	CLK_RST_CONTROLLER_RST_DEVL_SET_0,
	CLK_RST_CONTROLLER_RST_DEVH_SET_0,
	CLK_RST_CONTROLLER_RST_DEVU_SET_0,
	CLK_RST_CONTROLLER_RST_DEVV_SET_0,
	CLK_RST_CONTROLLER_RST_DEVW_SET_0,
	CLK_RST_CONTROLLER_RST_DEVX_SET_0,
	CLK_RST_CONTROLLER_RST_DEVY_SET_0,
};

static const uint32_t rst_dev_clr_regs[] = {
	CLK_RST_CONTROLLER_RST_DEVL_CLR_0,
	CLK_RST_CONTROLLER_RST_DEVH_CLR_0,
	CLK_RST_CONTROLLER_RST_DEVU_CLR_0,
	CLK_RST_CONTROLLER_RST_DEVV_CLR_0,
	CLK_RST_CONTROLLER_RST_DEVW_CLR_0,
	CLK_RST_CONTROLLER_RST_DEVX_CLR_0,
	CLK_RST_CONTROLLER_RST_DEVY_CLR_0,
};

static uint32_t clk_m_rate;
static const uint32_t pllp_rate = 408000000;

const struct tegra_clk tegra_clk_i2c_slow_data = {
	.rst_enb_id = (2 * 32) + 17,
	.reg = CLK_RST_CONTROLLER_CLK_SOURCE_I2C_SLOW_0,
	.div_bits = 8,
	.frac_width = 1,
	.clk_src_val = 6, /* clk_m */
	.parent_rate_ptr = &clk_m_rate,
};

const struct tegra_clk tegra_clk_i2c1_data = {
	.rst_enb_id = 12,
	.reg = CLK_RST_CONTROLLER_CLK_SOURCE_I2C1_0,
	.div_bits = 16,
	.frac_width = 0,
	.clk_src_val = 0, /* pllp_out0 */
	.parent_rate_ptr = &pllp_rate,
};

const struct tegra_clk tegra_clk_vi_i2c_data = {
	.rst_enb_id = (6 * 32) + 16,
	.reg = CLK_RST_CONTROLLER_CLK_SOURCE_VI_I2C_0,
	.div_bits = 16,
	.frac_width = 0,
	.clk_src_val = 0, /* pllp_out0 */
	.parent_rate_ptr = &pllp_rate,
};

int tegra_clk_init(void)
{
	uint32_t val;
	uint32_t osc_freq;

	val = readl(CLK_RST_CONTROLLER_BASE + CLK_RST_CONTROLLER_OSC_CTRL_0);

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

	return 0;
}

static int mask_write(const struct tegra_clk *clk, const uint32_t *regs, uint32_t num_regs)
{
	uint32_t regn = clk->rst_enb_id / 32;
	uint32_t bit = clk->rst_enb_id % 32;

	if (regn >= num_regs)
		return 1;

	writel(BIT(bit), CLK_RST_CONTROLLER_BASE + regs[regn]);

	return 0;
}

int tegra_clk_enable(const struct tegra_clk *clk)
{
	return mask_write(clk, clk_enb_set_regs, ARRAY_SIZE(clk_enb_set_regs));
}

int tegra_clk_disable(const struct tegra_clk *clk)
{
	return mask_write(clk, clk_enb_clr_regs, ARRAY_SIZE(clk_enb_clr_regs));
}

int tegra_clk_reset_set(const struct tegra_rst *rst)
{
	return mask_write((const struct tegra_clk *)rst, rst_dev_set_regs, ARRAY_SIZE(rst_dev_set_regs));
}

int tegra_clk_reset_clear(const struct tegra_rst *rst)
{
	return mask_write((const struct tegra_clk *)rst, rst_dev_clr_regs, ARRAY_SIZE(rst_dev_clr_regs));
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

	/*
	 * For now, we always round to ensure the clock runs no faster than
	 * the requested rate. If some clocks need other rounding options,
	 * we can add flags to struct tegra_clk for that.
	 */
	divisor = (((*clk->parent_rate_ptr) << clk->frac_width) + rate_hz - 1) / rate_hz;
	divisor -= (1 << clk->frac_width);
	if (divisor & ~NV_FIELD_MASK(CLK_RST_CONTROLLER_CLK_SOURCE_0_DIVISOR_RANGE))
		return 1;

	val = NV_DRF_NUM(CLK_RST_CONTROLLER, CLK_SOURCE, DIVISOR, divisor) |
		NV_DRF_NUM(CLK_RST_CONTROLLER, CLK_SOURCE, CLK_SRC, clk->clk_src_val);
	writel(val, CLK_RST_CONTROLLER_BASE + clk->reg);

	return 0;
}
