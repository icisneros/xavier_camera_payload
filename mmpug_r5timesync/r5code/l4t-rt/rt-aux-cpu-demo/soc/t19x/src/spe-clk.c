/*
 * Copyright (c) 2018 NVIDIA CORPORATION. All rights reserved.
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
#include <stdio.h>
#include <stdbool.h>

#include <FreeRTOS.h>

#include <address_map_new.h>
#include <arclk_rst.h>
#include <nvrm_drf.h>

#include <clk-tegra.h>
#include <clk-tegra-hw.h>
#include <delay.h>
#include <err-hook.h>
#include <macros.h>
#include <reg-access.h>

#include <spe-clk.h>

/* All frequencies are in kHz */

/* Not exact, but doesn't really matter */
#define CLK_S_RATE 33
#define PLLP_RATE 408000

#define DIV_MAX 254

#define ACTIVE_MODE_IDX 0
#define ACTIVE_IRQFIQ_MODE_IDX 1

#define PLLAON 0
#define PLLP 1

#define NFRAC_DIV (1 << 16)

#define PLLAON_TIMEOUT_US 300

struct pll {
	uint32_t base;
	uint32_t misc1;
	bool has_n_frac;
};

struct clk_src_data {
	uint32_t freq;
	uint32_t src_idx;
	struct pll *pll;
};

static uint32_t osc_freq;

static struct pll plls[] = {
	[PLLAON] = {
		.base = NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLAON_BASE_0,
		.misc1 = NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLAON_MISC_1_0,
		.has_n_frac = true,
	},
	[PLLP] = {
		.base = NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLP_BASE_0,
		.has_n_frac = false,
	},
};

static struct clk_src_data clk_srcs[] = {
	[CLK_SRC_PLLAON] = {
		.src_idx = NV_DRF_DEF(CLK_RST_CONTROLLER, CLK_SOURCE_AON_CPU_NIC,
				AON_CPU_NIC_CLK_SRC, pllAON_out),
		.pll = &plls[PLLAON],
	},
	[CLK_SRC_PLLP] = {
		.freq = PLLP_RATE,
		.src_idx = NV_DRF_DEF(CLK_RST_CONTROLLER, CLK_SOURCE_AON_CPU_NIC,
				AON_CPU_NIC_CLK_SRC, PLLP_OUT0),
		.pll = &plls[PLLP],
	},
	[CLK_SRC_OSC_UNDIV] = {
		.src_idx = NV_DRF_DEF(CLK_RST_CONTROLLER, CLK_SOURCE_AON_CPU_NIC,
				AON_CPU_NIC_CLK_SRC, OSC_UNDIV),
	},
	[CLK_SRC_CLK_S] = {
		.freq = CLK_S_RATE,
		.src_idx = NV_DRF_DEF(CLK_RST_CONTROLLER, CLK_SOURCE_AON_CPU_NIC,
				AON_CPU_NIC_CLK_SRC, CLK_S),
	},
};

struct clk {
	uint32_t base;
	uint32_t max_freq;
	uint32_t freq;
	uint32_t parent_freq;
	int src;
};

static struct clk clk_ids[] = {
	[CLK_ID_AON_CPU_NIC_ACTIVE] = {
		.base = NV_ADDRESS_MAP_CAR_BASE +
			CLK_RST_CONTROLLER_CLK_SOURCE_AON_CPU_NIC_TGT_ACTIVE_0,
		.max_freq = AON_CPU_MAX_RATE,
	},
	[CLK_ID_AON_CPU_NIC_ACTIVE_IRQFIQ] = {
	       .base = NV_ADDRESS_MAP_CAR_BASE +
			CLK_RST_CONTROLLER_CLK_SOURCE_AON_CPU_NIC_TGT_ACTIVE_IRQFIQ_0,
	       .max_freq = AON_CPU_MAX_RATE,
	},
	[CLK_ID_AON_CPU_NIC_IDLE_SHALLOW] = {
	       .base = NV_ADDRESS_MAP_CAR_BASE +
			CLK_RST_CONTROLLER_CLK_SOURCE_AON_CPU_NIC_TGT_IDLE_SHALLOW_0,
	       .max_freq = AON_CPU_MAX_RATE,
	},
	[CLK_ID_AON_CPU_NIC_IDLE_DEEP] = {
	       .base = NV_ADDRESS_MAP_CAR_BASE +
			CLK_RST_CONTROLLER_CLK_SOURCE_AON_CPU_NIC_TGT_IDLE_DEEP_0,
	       .max_freq = AON_CPU_MAX_RATE,
	},
	[CLK_ID_AON_CPU_NIC_STBY_SHALLOW] = {
	       .base = NV_ADDRESS_MAP_CAR_BASE +
			CLK_RST_CONTROLLER_CLK_SOURCE_AON_CPU_NIC_TGT_STBY_SHALLOW_0,
	       .max_freq = AON_CPU_MAX_RATE,
	},
	[CLK_ID_AON_CPU_NIC_STBY_DEEP] = {
	       .base = NV_ADDRESS_MAP_CAR_BASE +
			CLK_RST_CONTROLLER_CLK_SOURCE_AON_CPU_NIC_TGT_STBY_DEEP_0,
	       .max_freq = AON_CPU_MAX_RATE,
	},
	[CLK_ID_AON_CPU_NIC_DORMANT_SHALLOW] = {
	       .base = NV_ADDRESS_MAP_CAR_BASE +
			CLK_RST_CONTROLLER_CLK_SOURCE_AON_CPU_NIC_TGT_DORMANT_SHALLOW_0,
	       .max_freq = AON_CPU_MAX_RATE,
	},
	[CLK_ID_AON_CPU_NIC_DORMANT_DEEP] = {
	       .base = NV_ADDRESS_MAP_CAR_BASE +
			CLK_RST_CONTROLLER_CLK_SOURCE_AON_CPU_NIC_TGT_DORMANT_DEEP_0,
	       .max_freq = AON_CPU_MAX_RATE,
	},
	[CLK_ID_AON_APB_ACTIVE] = {
	       .base = NV_ADDRESS_MAP_CAR_BASE +
			CLK_RST_CONTROLLER_CLK_SOURCE_AON_APB_TGT_ACTIVE_0,
	       .max_freq = AON_APB_MAX_RATE,
	},
	[CLK_ID_AON_APB_ACTIVE_IRQFIQ] = {
	       .base = NV_ADDRESS_MAP_CAR_BASE +
			CLK_RST_CONTROLLER_CLK_SOURCE_AON_APB_TGT_ACTIVE_IRQFIQ_0,
	       .max_freq = AON_APB_MAX_RATE,
	},
	[CLK_ID_AON_APB_IDLE_SHALLOW] = {
	       .base = NV_ADDRESS_MAP_CAR_BASE +
			CLK_RST_CONTROLLER_CLK_SOURCE_AON_APB_TGT_IDLE_SHALLOW_0,
	       .max_freq = AON_APB_MAX_RATE,
	},
	[CLK_ID_AON_APB_IDLE_DEEP] = {
	       .base = NV_ADDRESS_MAP_CAR_BASE +
			CLK_RST_CONTROLLER_CLK_SOURCE_AON_APB_TGT_IDLE_DEEP_0,
	       .max_freq = AON_APB_MAX_RATE,
	},
	[CLK_ID_AON_APB_STBY_SHALLOW] = {
	       .base = NV_ADDRESS_MAP_CAR_BASE +
			CLK_RST_CONTROLLER_CLK_SOURCE_AON_APB_TGT_STBY_SHALLOW_0,
	       .max_freq = AON_APB_MAX_RATE,
	},
	[CLK_ID_AON_APB_STBY_DEEP] = {
	       .base = NV_ADDRESS_MAP_CAR_BASE +
			CLK_RST_CONTROLLER_CLK_SOURCE_AON_APB_TGT_STBY_DEEP_0,
	       .max_freq = AON_APB_MAX_RATE,
	},
	[CLK_ID_AON_APB_DORMANT_SHALLOW] = {
	       .base = NV_ADDRESS_MAP_CAR_BASE +
			CLK_RST_CONTROLLER_CLK_SOURCE_AON_APB_TGT_DORMANT_SHALLOW_0,
	       .max_freq = AON_APB_MAX_RATE,
	},
	[CLK_ID_AON_APB_DORMANT_DEEP] = {
	       .base = NV_ADDRESS_MAP_CAR_BASE +
			CLK_RST_CONTROLLER_CLK_SOURCE_AON_APB_TGT_DORMANT_DEEP_0,
	       .max_freq = AON_APB_MAX_RATE,
	},
	[CLK_ID_AON_CPU_NIC] = {
	       .base = NV_ADDRESS_MAP_CAR_BASE +
			CLK_RST_CONTROLLER_CLK_SOURCE_AON_CPU_NIC_0,
	       .max_freq = AON_CPU_MAX_RATE,
	},
	[CLK_ID_AON_APB] = {
	       .base = NV_ADDRESS_MAP_CAR_BASE +
			CLK_RST_CONTROLLER_CLK_SOURCE_AON_APB_0,
	       .max_freq = AON_APB_MAX_RATE,
	},
};

static uint32_t read_pll_freq(struct pll *pll)
{
	int32_t m, n, p, freq, n_frac;
	uint32_t val;

	val = readl(pll->base);
	p = NV_DRF_VAL(CLK_RST_CONTROLLER, PLLAON_BASE, PLLAON_DIVP, val);
	n = NV_DRF_VAL(CLK_RST_CONTROLLER, PLLAON_BASE, PLLAON_DIVN, val);
	m = NV_DRF_VAL(CLK_RST_CONTROLLER, PLLAON_BASE, PLLAON_DIVM, val);

	if (pll->has_n_frac) {
		val = readl(pll->misc1);
		val = NV_DRF_VAL(CLK_RST_CONTROLLER, PLLAON_MISC_1, PLLAON_DIVN_FRAC, val);
		/* sign extend from 16 to 32 bits */
		if (val & (1 << 15))
			n_frac = val | 0xffff0000;
		else
			n_frac = val;
	} else {
		n_frac = 0;
	}

	freq = (((int32_t)osc_freq * n) + (((int32_t)osc_freq * n_frac) /
				NFRAC_DIV)) / (m * p);

	return (uint32_t)freq;
}

void spe_clk_init(void)
{
	uint32_t val;
	uint32_t pll_aon_freq;

	val = readl(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_OSC_CTRL_0);
	val = NV_DRF_VAL(CLK_RST_CONTROLLER, OSC_CTRL, OSC_FREQ, val);

	switch (val) {
	case CLK_RST_CONTROLLER_OSC_CTRL_0_OSC_FREQ_OSC12:
		osc_freq = 12000;
		break;
	case CLK_RST_CONTROLLER_OSC_CTRL_0_OSC_FREQ_OSC13:
		osc_freq = 13000;
		break;
	case CLK_RST_CONTROLLER_OSC_CTRL_0_OSC_FREQ_OSC16P8:
		osc_freq = 16800;
		break;
	case CLK_RST_CONTROLLER_OSC_CTRL_0_OSC_FREQ_OSC19P2:
		osc_freq = 19200;
		break;
	case CLK_RST_CONTROLLER_OSC_CTRL_0_OSC_FREQ_OSC26:
		osc_freq = 26000;
		break;
	case CLK_RST_CONTROLLER_OSC_CTRL_0_OSC_FREQ_OSC38P4:
		osc_freq = 38400;
		break;
	case CLK_RST_CONTROLLER_OSC_CTRL_0_OSC_FREQ_OSC48:
		osc_freq = 48000;
		break;
	default:
		error_hook("Unsupported OSC frequency. Halting.");
		while(1);
	}

	clk_srcs[CLK_SRC_OSC_UNDIV].freq = osc_freq;

	pll_aon_freq = read_pll_freq(clk_srcs[CLK_SRC_PLLAON].pll);
	clk_srcs[CLK_SRC_PLLAON].freq = pll_aon_freq;
	tegra_parent_clk_set_rate(&tegra_parent_clk_pll_aon, pll_aon_freq);
}

int spe_clk_set_clk_freq(int clk_id, int clk_src, uint32_t freq)
{
	uint32_t parent_freq, src_idx;
	uint32_t div, val;
	struct clk *id;

	if (!freq)
		return -1;

	if (clk_id >= CLK_ID_MAX || clk_id < 0)
		return -1;

	if (clk_src >= CLK_SRC_MAX || clk_src < 0)
		return -1;

	id = &clk_ids[clk_id];
	parent_freq = clk_srcs[clk_src].freq;
	src_idx = clk_srcs[clk_src].src_idx;

	if (!parent_freq)
		return -1;

	if (freq == id->freq && clk_src == id->src &&
			parent_freq == id->parent_freq)
		return 0;

	if (freq > parent_freq)
		freq = parent_freq;

	if (freq > id->max_freq)
		freq = id->max_freq;

	if (freq < (parent_freq / ((DIV_MAX >> 1) + 1)))
		return -1;

	/* Round the actual frequency down */
	div = (parent_freq + freq - 1) / freq;

	/* The actual divisor is (N/2)+1, where N is the programmed value */
	div = (div - 1) << 1;

	if (div > DIV_MAX)
		div = DIV_MAX;

	id->freq = parent_freq / ((div >> 1) + 1);
	id->src = clk_src;
	id->parent_freq = parent_freq;

	val = NV_DRF_NUM(CLK_RST_CONTROLLER, CLK_SOURCE_AON_CPU_NIC,
				AON_CPU_NIC_CLK_DIVISOR, div) | src_idx;

	writel(val, id->base);
	return 0;
}

int spe_clk_set_clk_no_div(int clk_id, int clk_src)
{
	uint32_t freq;

	if (clk_src >= CLK_SRC_MAX || clk_src < 0)
		return -1;

	freq = clk_srcs[clk_src].freq;

	return spe_clk_set_clk_freq(clk_id, clk_src, freq);
}

void spe_clk_trigger_switch_fsm(void)
{
	uint32_t val;

	/* Switch current clock to ACTIVE_IRQFIQ clock */
	do {
		val = readl(NV_ADDRESS_MAP_CAR_BASE +
				CLK_RST_CONTROLLER_AON_SEQ_SW_INTERFACE_CTL_0);
		val = NV_DRF_VAL(CLK_RST_CONTROLLER, AON_SEQ_SW_INTERFACE_CTL,
				AON_SW_TGT_ACK, val);
	} while (val);
	val = NV_DRF_NUM(CLK_RST_CONTROLLER, AON_SEQ_SW_INTERFACE_CTL,
			AON_SW_TGT_STATE, ACTIVE_IRQFIQ_MODE_IDX) |
		NV_DRF_NUM(CLK_RST_CONTROLLER, AON_SEQ_SW_INTERFACE_CTL,
			AON_SW_TGT_REQ, 1);
	writel(val, NV_ADDRESS_MAP_CAR_BASE +
			CLK_RST_CONTROLLER_AON_SEQ_SW_INTERFACE_CTL_0);
	do {
		val = readl(NV_ADDRESS_MAP_CAR_BASE +
				CLK_RST_CONTROLLER_AON_SEQ_SW_INTERFACE_CTL_0);
		val = NV_DRF_VAL(CLK_RST_CONTROLLER, AON_SEQ_SW_INTERFACE_CTL,
				AON_SW_TGT_ACK, val);
	} while (!val);
	val = NV_DRF_NUM(CLK_RST_CONTROLLER, AON_SEQ_SW_INTERFACE_CTL,
			AON_SW_TGT_STATE, ACTIVE_IRQFIQ_MODE_IDX);
	writel(val, NV_ADDRESS_MAP_CAR_BASE +
			CLK_RST_CONTROLLER_AON_SEQ_SW_INTERFACE_CTL_0);
	do {
		val = readl(NV_ADDRESS_MAP_CAR_BASE +
				CLK_RST_CONTROLLER_AON_SEQ_SW_INTERFACE_CTL_0);
		val = NV_DRF_VAL(CLK_RST_CONTROLLER, AON_SEQ_SW_INTERFACE_CTL,
				AON_SW_TGT_ACK, val);
	} while (val);
	/* Switch current clock back to ACTIVE clock */
	val = NV_DRF_NUM(CLK_RST_CONTROLLER, AON_SEQ_SW_INTERFACE_CTL,
			AON_SW_TGT_STATE, ACTIVE_MODE_IDX) |
		NV_DRF_NUM(CLK_RST_CONTROLLER, AON_SEQ_SW_INTERFACE_CTL,
			AON_SW_TGT_REQ, 1);
	writel(val, NV_ADDRESS_MAP_CAR_BASE +
			CLK_RST_CONTROLLER_AON_SEQ_SW_INTERFACE_CTL_0);
	do {
		val = readl(NV_ADDRESS_MAP_CAR_BASE +
				CLK_RST_CONTROLLER_AON_SEQ_SW_INTERFACE_CTL_0);
		val = NV_DRF_VAL(CLK_RST_CONTROLLER, AON_SEQ_SW_INTERFACE_CTL,
				AON_SW_TGT_ACK, val);
	} while (!val);
	val = NV_DRF_NUM(CLK_RST_CONTROLLER, AON_SEQ_SW_INTERFACE_CTL,
			AON_SW_TGT_STATE, ACTIVE_MODE_IDX);
	writel(val, NV_ADDRESS_MAP_CAR_BASE +
			CLK_RST_CONTROLLER_AON_SEQ_SW_INTERFACE_CTL_0);
	do {
		val = readl(NV_ADDRESS_MAP_CAR_BASE +
				CLK_RST_CONTROLLER_AON_SEQ_SW_INTERFACE_CTL_0);
		val = NV_DRF_VAL(CLK_RST_CONTROLLER, AON_SEQ_SW_INTERFACE_CTL,
				AON_SW_TGT_ACK, val);
	} while (val);
}

static int wait_for_bit_set(uint32_t reg, uint32_t bit)
{
	uint64_t t_begin = get_time_ticks();
	uint32_t val;

	do {
		val = readl(reg);
		if (val & bit)
			return 0;
		udelay(1);

	} while (get_time_delta_us(t_begin) < PLLAON_TIMEOUT_US);

	return -1;
}

static int wait_for_pllaon_fll_lock(void)
{
	return wait_for_bit_set(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLAON_BASE_0,
			1 << CLK_RST_CONTROLLER_PLLAON_BASE_0_PLLAON_FREQ_LOCK_SHIFT);
}

static int wait_for_pllaon_pll_lock(void)
{
	return wait_for_bit_set(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLAON_BASE_0,
			1 << CLK_RST_CONTROLLER_PLLAON_BASE_0_PLLAON_LOCK_SHIFT);
}

void disable_pllaon(void)
{
	uint32_t val;

	val = readl(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLAON_BASE_0);
	val = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER, PLLAON_BASE, PLLAON_ENABLE,
			DISABLE, val);
	writel(val, NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLAON_BASE_0);

	val = readl(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLAON_MISC_1_0);
	val = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER, PLLAON_MISC_1, PLLAON_IDDQ,
			ON, val);
	writel(val, NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLAON_MISC_1_0);
}

void enable_pllaon(void)
{
	uint32_t val;

	val = readl(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLAON_MISC_1_0);
	val = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER, PLLAON_MISC_1, PLLAON_IDDQ,
			OFF, val);
	writel(val, NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLAON_MISC_1_0);

	udelay(2);

	val = readl(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLAON_BASE_0);
	val = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER, PLLAON_BASE, PLLAON_ENABLE,
			ENABLE, val);
	writel(val, NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLAON_BASE_0);

	/* wait for lock */
	if (wait_for_pllaon_fll_lock()) {
		error_hook("Fatal: pll_aon did not lock!");
		vPortEnterCritical();
		while (1);
	}

	/* wait for lock */
	if (wait_for_pllaon_pll_lock()) {
		error_hook("Fatal: pll_aon did not lock!");
		vPortEnterCritical();
		while (1);
	}
}
