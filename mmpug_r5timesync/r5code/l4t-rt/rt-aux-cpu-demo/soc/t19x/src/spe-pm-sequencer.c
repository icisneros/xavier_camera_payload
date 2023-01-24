/* Copyright (c) 2018-2020, NVIDIA CORPORATION. All rights reserved.
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

#include <stdio.h>
#include <stdint.h>

#include <address_map_new.h>
#include <arclk_rst.h>
#include <araopm.h>
#include <argpio_aon_sw.h>
#include <arpmc_impl.h>
#include <nvrm_drf.h>
#include <arwake.h>
#include <arscratch.h>

#include <debug.h>
#include <delay.h>
#include <err-hook.h>
#include <macros.h>
#include <reg-access.h>
#include <hsp-tegra.h>
#include <hsp-tegra-hw.h>

#include <bpmp-ipc.h>
#include <bpmp-ipc-protocol.h>
#include <combined-uart.h>
#include <spe-clk.h>
#include <spe-pm.h>
#include <spe-vic.h>

#define RAM_PG_CTRL_OFFSET 0
#define RAM_PG_STATUS_OFFSET (PMC_IMPL_PART_AOPG_CACHE_POWER_GATE_STATUS_0 - \
				PMC_IMPL_PART_AOPG_CACHE_POWER_GATE_CONTROL_0)
#define RAM_CLAMP_CTRL_OFFSET (PMC_IMPL_PART_AOPG_CACHE_CLAMP_CONTROL_0 - \
				PMC_IMPL_PART_AOPG_CACHE_POWER_GATE_CONTROL_0)

#define aopm_addr(offset) (NV_ADDRESS_MAP_AON_PM_BASE + offset)

static enum pm_ram_pg_status cache_pg_state = PM_RAM_ON;

static bool pllaon_disabled = false;

struct reg_value {
	uint32_t addr;
	uint32_t mask;
	uint32_t value;
};

struct clk_setting {
	uint32_t clk_id;
	uint32_t clk_src;
	/* 0 denotes max freq possible for a given clk_src */
	uint32_t freq;
};

#if PM_USE_HW_SEQUENCER_FOR_PLLAON
/* addr, mask, val */
static struct reg_value pllaon_lp_config_on[] = {
	{
		NV_ADDRESS_MAP_CAR_BASE +
			CLK_RST_CONTROLLER_PLL_CFG_AON_TGT_ACTIVE_0,

		CLK_RST_CONTROLLER_PLL_CFG_AON_TGT_ACTIVE_0_AON_PLL_ACTIVE_PLL_IDDQ_FIELD |
		CLK_RST_CONTROLLER_PLL_CFG_AON_TGT_ACTIVE_0_AON_PLL_ACTIVE_PLL_ENABLE_FIELD,

		NV_DRF_NUM(CLK_RST_CONTROLLER, PLL_CFG_AON_TGT_ACTIVE,
			AON_PLL_ACTIVE_PLL_IDDQ, 0) |
		NV_DRF_NUM(CLK_RST_CONTROLLER, PLL_CFG_AON_TGT_ACTIVE,
			AON_PLL_ACTIVE_PLL_ENABLE, 1)
	},
	{
		NV_ADDRESS_MAP_CAR_BASE +
			CLK_RST_CONTROLLER_PLL_CFG_AON_TGT_ACTIVE_IRQFIQ_0,

		CLK_RST_CONTROLLER_PLL_CFG_AON_TGT_ACTIVE_IRQFIQ_0_AON_PLL_ACTIVE_IRQFIQ_PLL_IDDQ_FIELD |
		CLK_RST_CONTROLLER_PLL_CFG_AON_TGT_ACTIVE_IRQFIQ_0_AON_PLL_ACTIVE_IRQFIQ_PLL_ENABLE_FIELD,

		NV_DRF_NUM(CLK_RST_CONTROLLER, PLL_CFG_AON_TGT_ACTIVE_IRQFIQ,
			AON_PLL_ACTIVE_IRQFIQ_PLL_IDDQ, 0) |
		NV_DRF_NUM(CLK_RST_CONTROLLER, PLL_CFG_AON_TGT_ACTIVE_IRQFIQ,
			AON_PLL_ACTIVE_IRQFIQ_PLL_ENABLE, 1)
	},
	{
		NV_ADDRESS_MAP_CAR_BASE +
			CLK_RST_CONTROLLER_PLL_CFG_AON_TGT_IDLE_SHALLOW_0,

		CLK_RST_CONTROLLER_PLL_CFG_AON_TGT_IDLE_SHALLOW_0_AON_PLL_IDLE_SHALLOW_PLL_IDDQ_FIELD |
		CLK_RST_CONTROLLER_PLL_CFG_AON_TGT_IDLE_SHALLOW_0_AON_PLL_IDLE_SHALLOW_PLL_ENABLE_FIELD,

		NV_DRF_NUM(CLK_RST_CONTROLLER, PLL_CFG_AON_TGT_IDLE_SHALLOW,
			AON_PLL_IDLE_SHALLOW_PLL_IDDQ, 0) |
		NV_DRF_NUM(CLK_RST_CONTROLLER, PLL_CFG_AON_TGT_IDLE_SHALLOW,
			AON_PLL_IDLE_SHALLOW_PLL_ENABLE, 1)
	},
	{
		NV_ADDRESS_MAP_CAR_BASE +
			CLK_RST_CONTROLLER_PLL_CFG_AON_TGT_IDLE_DEEP_0,

		CLK_RST_CONTROLLER_PLL_CFG_AON_TGT_IDLE_DEEP_0_AON_PLL_IDLE_DEEP_PLL_IDDQ_FIELD |
		CLK_RST_CONTROLLER_PLL_CFG_AON_TGT_IDLE_DEEP_0_AON_PLL_IDLE_DEEP_PLL_ENABLE_FIELD,

		NV_DRF_NUM(CLK_RST_CONTROLLER, PLL_CFG_AON_TGT_IDLE_DEEP,
			AON_PLL_IDLE_DEEP_PLL_IDDQ, 1) |
		NV_DRF_NUM(CLK_RST_CONTROLLER, PLL_CFG_AON_TGT_IDLE_DEEP,
			AON_PLL_IDLE_DEEP_PLL_ENABLE, 0)
	},
	{
		NV_ADDRESS_MAP_CAR_BASE +
			CLK_RST_CONTROLLER_PLL_CFG_AON_TGT_STBY_SHALLOW_0,

		CLK_RST_CONTROLLER_PLL_CFG_AON_TGT_STBY_SHALLOW_0_AON_PLL_STBY_SHALLOW_PLL_IDDQ_FIELD |
		CLK_RST_CONTROLLER_PLL_CFG_AON_TGT_STBY_SHALLOW_0_AON_PLL_STBY_SHALLOW_PLL_ENABLE_FIELD,

		NV_DRF_NUM(CLK_RST_CONTROLLER, PLL_CFG_AON_TGT_STBY_SHALLOW,
			AON_PLL_STBY_SHALLOW_PLL_IDDQ, 0) |
		NV_DRF_NUM(CLK_RST_CONTROLLER, PLL_CFG_AON_TGT_STBY_SHALLOW,
			AON_PLL_STBY_SHALLOW_PLL_ENABLE, 1)
	},
	{
		NV_ADDRESS_MAP_CAR_BASE +
			CLK_RST_CONTROLLER_PLL_CFG_AON_TGT_STBY_DEEP_0,

		CLK_RST_CONTROLLER_PLL_CFG_AON_TGT_STBY_DEEP_0_AON_PLL_STBY_DEEP_PLL_IDDQ_FIELD |
		CLK_RST_CONTROLLER_PLL_CFG_AON_TGT_STBY_DEEP_0_AON_PLL_STBY_DEEP_PLL_ENABLE_FIELD,

		NV_DRF_NUM(CLK_RST_CONTROLLER, PLL_CFG_AON_TGT_STBY_DEEP,
			AON_PLL_STBY_DEEP_PLL_IDDQ, 1) |
		NV_DRF_NUM(CLK_RST_CONTROLLER, PLL_CFG_AON_TGT_STBY_DEEP,
			AON_PLL_STBY_DEEP_PLL_ENABLE, 0)
	},
	{
		NV_ADDRESS_MAP_CAR_BASE +
			CLK_RST_CONTROLLER_PLL_CFG_AON_TGT_DORMANT_SHALLOW_0,

		CLK_RST_CONTROLLER_PLL_CFG_AON_TGT_DORMANT_SHALLOW_0_AON_PLL_DORMANT_SHALLOW_PLL_IDDQ_FIELD |
		CLK_RST_CONTROLLER_PLL_CFG_AON_TGT_DORMANT_SHALLOW_0_AON_PLL_DORMANT_SHALLOW_PLL_ENABLE_FIELD,

		NV_DRF_NUM(CLK_RST_CONTROLLER, PLL_CFG_AON_TGT_DORMANT_SHALLOW,
			AON_PLL_DORMANT_SHALLOW_PLL_IDDQ, 0) |
		NV_DRF_NUM(CLK_RST_CONTROLLER, PLL_CFG_AON_TGT_DORMANT_SHALLOW,
			AON_PLL_DORMANT_SHALLOW_PLL_ENABLE, 1)
	},
	{
		NV_ADDRESS_MAP_CAR_BASE +
			CLK_RST_CONTROLLER_PLL_CFG_AON_TGT_DORMANT_DEEP_0,

		CLK_RST_CONTROLLER_PLL_CFG_AON_TGT_DORMANT_DEEP_0_AON_PLL_DORMANT_DEEP_PLL_IDDQ_FIELD |
		CLK_RST_CONTROLLER_PLL_CFG_AON_TGT_DORMANT_DEEP_0_AON_PLL_DORMANT_DEEP_PLL_ENABLE_FIELD,

		NV_DRF_NUM(CLK_RST_CONTROLLER, PLL_CFG_AON_TGT_DORMANT_DEEP,
			AON_PLL_DORMANT_DEEP_PLL_IDDQ, 1) |
		NV_DRF_NUM(CLK_RST_CONTROLLER, PLL_CFG_AON_TGT_DORMANT_DEEP,
			AON_PLL_DORMANT_DEEP_PLL_ENABLE, 0)
	},
};

static struct reg_value pllaon_lp_config_off[] = {
	{
		NV_ADDRESS_MAP_CAR_BASE +
			CLK_RST_CONTROLLER_PLL_CFG_AON_TGT_IDLE_SHALLOW_0,

		CLK_RST_CONTROLLER_PLL_CFG_AON_TGT_IDLE_SHALLOW_0_AON_PLL_IDLE_SHALLOW_PLL_IDDQ_FIELD |
		CLK_RST_CONTROLLER_PLL_CFG_AON_TGT_IDLE_SHALLOW_0_AON_PLL_IDLE_SHALLOW_PLL_ENABLE_FIELD,

		NV_DRF_NUM(CLK_RST_CONTROLLER, PLL_CFG_AON_TGT_IDLE_SHALLOW,
			AON_PLL_IDLE_SHALLOW_PLL_IDDQ, 1) |
		NV_DRF_NUM(CLK_RST_CONTROLLER, PLL_CFG_AON_TGT_IDLE_SHALLOW,
			AON_PLL_IDLE_SHALLOW_PLL_ENABLE, 0)
	},
	{
		NV_ADDRESS_MAP_CAR_BASE +
			CLK_RST_CONTROLLER_PLL_CFG_AON_TGT_STBY_SHALLOW_0,

		CLK_RST_CONTROLLER_PLL_CFG_AON_TGT_STBY_SHALLOW_0_AON_PLL_STBY_SHALLOW_PLL_IDDQ_FIELD |
		CLK_RST_CONTROLLER_PLL_CFG_AON_TGT_STBY_SHALLOW_0_AON_PLL_STBY_SHALLOW_PLL_ENABLE_FIELD,

		NV_DRF_NUM(CLK_RST_CONTROLLER, PLL_CFG_AON_TGT_STBY_SHALLOW,
			AON_PLL_STBY_SHALLOW_PLL_IDDQ, 1) |
		NV_DRF_NUM(CLK_RST_CONTROLLER, PLL_CFG_AON_TGT_STBY_SHALLOW,
			AON_PLL_STBY_SHALLOW_PLL_ENABLE, 0)
	},
	{
		NV_ADDRESS_MAP_CAR_BASE +
			CLK_RST_CONTROLLER_PLL_CFG_AON_TGT_DORMANT_SHALLOW_0,

		CLK_RST_CONTROLLER_PLL_CFG_AON_TGT_DORMANT_SHALLOW_0_AON_PLL_DORMANT_SHALLOW_PLL_IDDQ_FIELD |
		CLK_RST_CONTROLLER_PLL_CFG_AON_TGT_DORMANT_SHALLOW_0_AON_PLL_DORMANT_SHALLOW_PLL_ENABLE_FIELD,

		NV_DRF_NUM(CLK_RST_CONTROLLER, PLL_CFG_AON_TGT_DORMANT_SHALLOW,
			AON_PLL_DORMANT_SHALLOW_PLL_IDDQ, 1) |
		NV_DRF_NUM(CLK_RST_CONTROLLER, PLL_CFG_AON_TGT_DORMANT_SHALLOW,
			AON_PLL_DORMANT_SHALLOW_PLL_ENABLE, 0)
	},
};
#endif

/* addr, mask, val */
static struct reg_value aopm_config[] = {
	{
		NV_ADDRESS_MAP_AON_PM_BASE + AOPM_EXIT_MASK_0,
		AOPM_EXIT_MASK_0_WRITE_MASK,
		NV_DRF_DEF(AOPM, EXIT_MASK, CLKSTOPPED, MASK) |
			NV_DRF_DEF(AOPM, EXIT_MASK, WFE, MASK) |
			NV_DRF_DEF(AOPM, EXIT_MASK, DBG_TIMEOUT, MASK)
	},
	{
		NV_ADDRESS_MAP_AON_PM_BASE + AOPM_TCM_RET_TIMER_0,
		AOPM_TCM_RET_TIMER_0_WRITE_MASK,
		/* This has to be 0 due to HW bug 200108439 */
		0
	},
};

static struct clk_setting clk_config_active[] = {
	{
		.clk_id = CLK_ID_AON_CPU_NIC_ACTIVE,
		.clk_src = CLK_SRC_PLLAON,
		.freq = AON_CPU_MAX_RATE / 2,
	},
	{
		.clk_id = CLK_ID_AON_APB_ACTIVE,
		.clk_src = CLK_SRC_PLLAON,
		.freq = AON_APB_MAX_RATE,
	},
	{
		.clk_id = CLK_ID_AON_CPU_NIC_ACTIVE_IRQFIQ,
		.clk_src = CLK_SRC_PLLAON,
		.freq = AON_CPU_MAX_RATE / 2,
	},
	{
		.clk_id = CLK_ID_AON_APB_ACTIVE_IRQFIQ,
		.clk_src = CLK_SRC_PLLAON,
		.freq = AON_APB_MAX_RATE,
	},
};

static struct clk_setting clk_config_shallow[] = {
	{
		.clk_id = CLK_ID_AON_CPU_NIC_IDLE_SHALLOW,
		.clk_src = CLK_SRC_PLLAON,
		.freq = AON_CPU_MAX_RATE / 2,
	},
	{
		.clk_id = CLK_ID_AON_CPU_NIC_STBY_SHALLOW,
		.clk_src = CLK_SRC_PLLAON,
		.freq = AON_CPU_MAX_RATE / 2,
	},
	{
		.clk_id = CLK_ID_AON_CPU_NIC_DORMANT_SHALLOW,
		.clk_src = CLK_SRC_OSC_UNDIV,
		.freq = 0,
	},
	{
		.clk_id = CLK_ID_AON_APB_IDLE_SHALLOW,
		.clk_src = CLK_SRC_PLLAON,
		.freq = AON_APB_MAX_RATE,
	},
	{
		.clk_id = CLK_ID_AON_APB_STBY_SHALLOW,
		.clk_src = CLK_SRC_PLLAON,
		.freq = AON_APB_MAX_RATE,
	},
	{
		.clk_id = CLK_ID_AON_APB_DORMANT_SHALLOW,
		.clk_src = CLK_SRC_OSC_UNDIV,
		.freq = 0,
	},
};

static struct clk_setting clk_config_deep[] = {
	{
		.clk_id = CLK_ID_AON_CPU_NIC_IDLE_DEEP,
		.clk_src = CLK_SRC_OSC_UNDIV,
		.freq = 0,
	},
	{
		.clk_id = CLK_ID_AON_CPU_NIC_STBY_DEEP,
		.clk_src = CLK_SRC_OSC_UNDIV,
		.freq = 0,
	},
	{
		.clk_id = CLK_ID_AON_CPU_NIC_DORMANT_DEEP,
		.clk_src = CLK_SRC_CLK_S,
		.freq = 0,
	},
	{
		.clk_id = CLK_ID_AON_APB_IDLE_DEEP,
		.clk_src = CLK_SRC_OSC_UNDIV,
		.freq = 0,
	},
	{
		.clk_id = CLK_ID_AON_APB_STBY_DEEP,
		.clk_src = CLK_SRC_OSC_UNDIV,
		.freq = 0,
	},
	{
		.clk_id = CLK_ID_AON_APB_DORMANT_DEEP,
		.clk_src = CLK_SRC_CLK_S,
		.freq = 0,
	},
};

static struct clk_setting clk_config_pllp[] = {
	{
		.clk_id = CLK_ID_AON_CPU_NIC_IDLE_SHALLOW,
		.clk_src = CLK_SRC_PLLP,
		.freq = AON_CPU_MAX_RATE / 4,
	},
	{
		.clk_id = CLK_ID_AON_CPU_NIC_STBY_SHALLOW,
		.clk_src = CLK_SRC_PLLP,
		.freq = AON_CPU_MAX_RATE / 4,
	},
	{
		.clk_id = CLK_ID_AON_APB_IDLE_SHALLOW,
		.clk_src = CLK_SRC_PLLP,
		.freq = AON_APB_MAX_RATE / 2,
	},
	{
		.clk_id = CLK_ID_AON_APB_STBY_SHALLOW,
		.clk_src = CLK_SRC_PLLP,
		.freq = AON_APB_MAX_RATE / 2,
	},
};

#if !PM_USE_HW_SEQUENCER_FOR_PLLAON
static struct clk_setting clk_config_osc_undiv[] = {
	{
		.clk_id = CLK_ID_AON_CPU_NIC_ACTIVE,
		.clk_src = CLK_SRC_OSC_UNDIV,
		.freq = 0,
	},
	{
		.clk_id = CLK_ID_AON_CPU_NIC_ACTIVE_IRQFIQ,
		.clk_src = CLK_SRC_OSC_UNDIV,
		.freq = 0,
	},
	{
		.clk_id = CLK_ID_AON_CPU_NIC_IDLE_SHALLOW,
		.clk_src = CLK_SRC_OSC_UNDIV,
		.freq = 0,
	},
	{
		.clk_id = CLK_ID_AON_CPU_NIC_STBY_SHALLOW,
		.clk_src = CLK_SRC_OSC_UNDIV,
		.freq = 0,
	},
	{
		.clk_id = CLK_ID_AON_CPU_NIC_DORMANT_SHALLOW,
		.clk_src = CLK_SRC_OSC_UNDIV,
		.freq = 0,
	},
	{
		.clk_id = CLK_ID_AON_APB_ACTIVE,
		.clk_src = CLK_SRC_OSC_UNDIV,
		.freq = 0,
	},
	{
		.clk_id = CLK_ID_AON_APB_ACTIVE_IRQFIQ,
		.clk_src = CLK_SRC_OSC_UNDIV,
		.freq = 0,
	},
	{
		.clk_id = CLK_ID_AON_APB_IDLE_SHALLOW,
		.clk_src = CLK_SRC_OSC_UNDIV,
		.freq = 0,
	},
	{
		.clk_id = CLK_ID_AON_APB_STBY_SHALLOW,
		.clk_src = CLK_SRC_OSC_UNDIV,
		.freq = 0,
	},
	{
		.clk_id = CLK_ID_AON_APB_DORMANT_SHALLOW,
		.clk_src = CLK_SRC_OSC_UNDIV,
		.freq = 0,
	},
};
#endif

/*
 * Programs len number of registers in the given array.
 */
static void program_regs(struct reg_value *seq, unsigned int len)
{
	uint32_t val;

	while (len--) {
		val = readl(seq->addr);
		val = (val & ~seq->mask) | seq->value;
		writel(val, seq->addr);
		seq++;
	}
}

static int program_clk_config(struct clk_setting *c, unsigned int len)
{
	int ret = 0;
	unsigned int i;

	for (i = 0; i < len; i++) {
		if (c[i].freq)
			ret = spe_clk_set_clk_freq(c[i].clk_id, c[i].clk_src,
							c[i].freq);
		else
			ret = spe_clk_set_clk_no_div(c[i].clk_id, c[i].clk_src);

		if (ret)
			return ret;
	}

	return ret;
}

static enum pm_ram_pg_status pm_ram_powergate(uint32_t base)
{
	uint32_t val;

	/* Assert clamps */
	val = NV_DRF_DEF(PMC_IMPL, PART_AOPG_CACHE_CLAMP_CONTROL, CLAMP, ON);
	writel(val, base + RAM_CLAMP_CTRL_OFFSET);
	udelay(2);

	/* Start PG */
	val = NV_DRF_DEF(PMC_IMPL, PART_AOPG_CACHE_POWER_GATE_CONTROL,
			INTER_PART_DELAY_EN, ENABLE);
	val = NV_FLD_SET_DRF_DEF(PMC_IMPL, PART_AOPG_CACHE_POWER_GATE_CONTROL,
			SRAM_SLEEP_EN, ON, val);
	val = NV_FLD_SET_DRF_DEF(PMC_IMPL, PART_AOPG_CACHE_POWER_GATE_CONTROL,
			LOGIC_SLEEP, ON, val);
	val = NV_FLD_SET_DRF_DEF(PMC_IMPL, PART_AOPG_CACHE_POWER_GATE_CONTROL,
			START, PENDING, val);
	writel(val, base + RAM_PG_CTRL_OFFSET);

	/* Wait until PG is done */
	do {
		val = readl(base + RAM_PG_CTRL_OFFSET);
		val = NV_DRF_VAL(PMC_IMPL, PART_AOPG_CACHE_POWER_GATE_CONTROL,
				START, val);
	} while (val != PMC_IMPL_PART_AOPG_CACHE_POWER_GATE_CONTROL_0_START_DONE);

	/* Verify status */
	val = readl(base + RAM_PG_STATUS_OFFSET);
	val = NV_DRF_VAL(PMC_IMPL, PART_AOPG_CACHE_POWER_GATE_STATUS,
				SRAM_SLEEP_STS, val);
	if (val == PMC_IMPL_PART_AOPG_CACHE_POWER_GATE_STATUS_0_SRAM_SLEEP_STS_ON)
		return PM_RAM_OFF;
	else
		return PM_RAM_ERROR;
}

static enum pm_ram_pg_status pm_ram_unpowergate(uint32_t base)
{
	uint32_t val;

	/* Start un-PG */
	val = NV_DRF_DEF(PMC_IMPL, PART_AOPG_CACHE_POWER_GATE_CONTROL,
			INTER_PART_DELAY_EN, ENABLE);
	val = NV_FLD_SET_DRF_DEF(PMC_IMPL, PART_AOPG_CACHE_POWER_GATE_CONTROL,
			START, PENDING, val);
	writel(val, base + RAM_PG_CTRL_OFFSET);

	/* Wait until un-PG is done */
	do {
		val = readl(base + RAM_PG_CTRL_OFFSET);
		val = NV_DRF_VAL(PMC_IMPL, PART_AOPG_CACHE_POWER_GATE_CONTROL,
				START, val);
	} while (val != PMC_IMPL_PART_AOPG_CACHE_POWER_GATE_CONTROL_0_START_DONE);

	/* Verify status */
	val = readl(base + RAM_PG_STATUS_OFFSET);
	val = NV_DRF_VAL(PMC_IMPL, PART_AOPG_CACHE_POWER_GATE_STATUS,
				SRAM_SLEEP_STS, val);
	if (val != PMC_IMPL_PART_AOPG_CACHE_POWER_GATE_STATUS_0_SRAM_SLEEP_STS_OFF)
		return PM_RAM_ERROR;

	/* De-assert clamps */
	val = NV_DRF_DEF(PMC_IMPL, PART_AOPG_CACHE_CLAMP_CONTROL, CLAMP, OFF);
	writel(val, base + RAM_CLAMP_CTRL_OFFSET);
	udelay(2);

	return PM_RAM_ON;
}

void pm_hw_init(void)
{
	uint32_t val;

	val = readl(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLAON_MISC_0_0);
	val = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLAON_MISC_0, PLLAON_RESET,
			0, val);
	writel(val, NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLAON_MISC_0_0);

	spe_clk_init();

	/* Disable burst clk change on IRQ for now */
	writel(0, aopm_addr(AOPM_BURST_CLK_0));

	val = readl(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLAON_MISC_4_0);
	val = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLAON_MISC_4, PLLAON_SEL_IREF,
			1, val);
	writel(val, NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLAON_MISC_4_0);

	spe_clk_set_clk_no_div(CLK_ID_AON_CPU_NIC, CLK_SRC_OSC_UNDIV);
	spe_clk_set_clk_no_div(CLK_ID_AON_APB, CLK_SRC_OSC_UNDIV);

#if PM_USE_HW_SEQUENCER_FOR_PLLAON
	udelay(2);

	disable_pllaon();

	udelay(5);

	program_regs(pllaon_lp_config_on, ARRAY_SIZE(pllaon_lp_config_on));
#endif

	program_clk_config(clk_config_active, ARRAY_SIZE(clk_config_active));
	program_clk_config(clk_config_shallow, ARRAY_SIZE(clk_config_shallow));
	program_clk_config(clk_config_deep, ARRAY_SIZE(clk_config_deep));
	program_regs(aopm_config, ARRAY_SIZE(aopm_config));

#if PM_USE_HW_SEQUENCER_FOR_PLLAON
	val = readl(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLAON_BASE_0);
	val = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLAON_BASE, PLLAON_OVERRIDE,
			0, val);
	writel(val, NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLAON_BASE_0);
#endif

	val = NV_DRF_NUM(CLK_RST_CONTROLLER, AON_SEQ_CTL_0,
			AON_SWDIV_SEQUENCER_EN, 1);
#if PM_USE_HW_SEQUENCER_FOR_PLLAON
	val = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, AON_SEQ_CTL_0,
			AON_PLL_SEQUENCER_EN, 1, val);
#else
	val = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, AON_SEQ_CTL_0,
			AON_SWDIV_BYP_PLL_CONFIG, 1, val);
	val = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, AON_SEQ_CTL_0,
			AON_DISABLE_PLL_HW_CONTROL, 1, val);
#endif
	writel(val, NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_AON_SEQ_CTL_0_0);

	udelay(2);

	spe_clk_trigger_switch_fsm();
}

#if !PM_USE_HW_SEQUENCER_FOR_PLLAON
static void switch_aon_clks_to_osc_undiv(void)
{
	program_clk_config(clk_config_osc_undiv, ARRAY_SIZE(clk_config_osc_undiv));
	spe_clk_trigger_switch_fsm();
}

static void restore_aon_clks(void)
{
	program_clk_config(clk_config_active, ARRAY_SIZE(clk_config_active));
	program_clk_config(clk_config_shallow, ARRAY_SIZE(clk_config_shallow));
	spe_clk_trigger_switch_fsm();
}
#endif

void pm_turn_off_caches(void)
{
	if (cache_pg_state != PM_RAM_ON)
		return;

	cache_pg_state = pm_ram_powergate(NV_ADDRESS_MAP_PMC_IMPL_BASE +
				PMC_IMPL_PART_AOPG_CACHE_POWER_GATE_CONTROL_0);
}

void pm_turn_on_caches(void)
{
	if (cache_pg_state != PM_RAM_OFF)
		return;

	cache_pg_state = pm_ram_unpowergate(NV_ADDRESS_MAP_PMC_IMPL_BASE +
				PMC_IMPL_PART_AOPG_CACHE_POWER_GATE_CONTROL_0);
}

enum pm_ram_pg_status get_cache_power_status(void)
{
	return cache_pg_state;
}


void pm_suspend_sc7(void)
{
	if (pllaon_disabled) {
#if PM_USE_HW_SEQUENCER_FOR_PLLAON
		program_regs(pllaon_lp_config_on,
				ARRAY_SIZE(pllaon_lp_config_on));
#endif
		program_clk_config(clk_config_shallow, ARRAY_SIZE(clk_config_shallow));
		spe_clk_trigger_switch_fsm();
	}
}

void pm_resume_sc7(void)
{
	if (pllaon_disabled) {
		program_clk_config(clk_config_pllp,
				ARRAY_SIZE(clk_config_pllp));
#if PM_USE_HW_SEQUENCER_FOR_PLLAON
		program_regs(pllaon_lp_config_off,
				ARRAY_SIZE(pllaon_lp_config_off));
#endif
		spe_clk_trigger_switch_fsm();
	}
}
