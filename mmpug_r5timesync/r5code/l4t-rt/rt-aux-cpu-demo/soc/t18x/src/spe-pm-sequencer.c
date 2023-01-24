/* Copyright (c) 2015-2017 NVIDIA CORPORATION.  All rights reserved.
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
#include <arpmc_impl.h>
#include <nvrm_drf.h>
#include <arwake.h>
#include <arscratch.h>

#include <macros.h>
#include <reg-access.h>
#include <delay.h>

#include <spe-pm.h>
#include <spe-vic.h>

#include <debug.h>
#include <err-hook.h>

#include <argpio_aon_sw.h>
#include <clk.h>

static enum pm_ram_pg_status cache_pg_state = PM_RAM_ON;
#define TIMEOUT_USEC 1

void pm_turn_off_caches(void)
{
	uint32_t val, tcurr;

	if (cache_pg_state != PM_RAM_ON)
		return;

	/* Assert clamps */
	val = NV_DRF_DEF(PMC_IMPL, PART_AOPG_CACHE_CLAMP_CONTROL, CLAMP, ON);
	writel(val, NV_ADDRESS_MAP_PMC_IMPL_BASE +
				PMC_IMPL_PART_AOPG_CACHE_CLAMP_CONTROL_0);
	udelay(2);

	val = readl(NV_ADDRESS_MAP_PMC_IMPL_BASE +
				PMC_IMPL_PART_AOPG_CACHE_POWER_GATE_CONTROL_0);
	val = NV_FLD_SET_DRF_DEF(PMC_IMPL, PART_AOPG_CACHE_POWER_GATE_CONTROL,
			SRAM_SD, ON, val);
	val = NV_FLD_SET_DRF_DEF(PMC_IMPL, PART_AOPG_CACHE_POWER_GATE_CONTROL,
			START, PENDING, val);
	writel(val, NV_ADDRESS_MAP_PMC_IMPL_BASE +
				PMC_IMPL_PART_AOPG_CACHE_POWER_GATE_CONTROL_0);

	/* Wait until PG is done */
	do {
		val = readl(NV_ADDRESS_MAP_PMC_IMPL_BASE +
				PMC_IMPL_PART_AOPG_CACHE_POWER_GATE_CONTROL_0);
		val = NV_DRF_VAL(PMC_IMPL, PART_AOPG_CACHE_POWER_GATE_CONTROL,
				START, val);
		tcurr = get_time_ticks();
	} while (val != PMC_IMPL_PART_AOPG_CACHE_POWER_GATE_CONTROL_0_START_DONE &&
		((get_time_ticks() - tcurr) <= TIMEOUT_USEC));

	/* Verify status */
	val = readl(NV_ADDRESS_MAP_PMC_IMPL_BASE +
				PMC_IMPL_PART_AOPG_CACHE_POWER_GATE_STATUS_0);
	val = NV_DRF_VAL(PMC_IMPL, PART_AOPG_CACHE_POWER_GATE_STATUS,
				SRAM_SD_STS, val);
	if (val == PMC_IMPL_PART_AOPG_CACHE_POWER_GATE_STATUS_0_SRAM_SD_STS_ON) {
		cache_pg_state = PM_RAM_OFF;
	} else {
		cache_pg_state = PM_RAM_ERROR;
		error_hook("Cache in error state \r\n");
	}
}

void pm_turn_on_caches(void)
{
	uint32_t val, tcurr;

	if (cache_pg_state != PM_RAM_OFF)
		return;

	val = readl(NV_ADDRESS_MAP_PMC_IMPL_BASE +
				PMC_IMPL_PART_AOPG_CACHE_POWER_GATE_CONTROL_0);
	val = NV_FLD_SET_DRF_DEF(PMC_IMPL, PART_AOPG_CACHE_POWER_GATE_CONTROL,
			SRAM_SD, OFF, val);
	val = NV_FLD_SET_DRF_DEF(PMC_IMPL, PART_AOPG_CACHE_POWER_GATE_CONTROL,
			START, PENDING, val);
	writel(val, NV_ADDRESS_MAP_PMC_IMPL_BASE +
				PMC_IMPL_PART_AOPG_CACHE_POWER_GATE_CONTROL_0);

	/* Wait until un-PG is done */
	do {
		val = readl(NV_ADDRESS_MAP_PMC_IMPL_BASE +
				PMC_IMPL_PART_AOPG_CACHE_POWER_GATE_CONTROL_0);
		val = NV_DRF_VAL(PMC_IMPL, PART_AOPG_CACHE_POWER_GATE_CONTROL,
				START, val);
		tcurr = get_time_ticks();
	} while (val != PMC_IMPL_PART_AOPG_CACHE_POWER_GATE_CONTROL_0_START_DONE &&
		((get_time_ticks() - tcurr) <= TIMEOUT_USEC));

	/* Verify status */
	val = readl(NV_ADDRESS_MAP_PMC_IMPL_BASE +
				PMC_IMPL_PART_AOPG_CACHE_POWER_GATE_STATUS_0);
	val = NV_DRF_VAL(PMC_IMPL, PART_AOPG_CACHE_POWER_GATE_STATUS,
				SRAM_SD_STS, val);
	if (val == PMC_IMPL_PART_AOPG_CACHE_POWER_GATE_STATUS_0_SRAM_SD_STS_OFF) {
		cache_pg_state = PM_RAM_ON;
	} else {
		cache_pg_state = PM_RAM_ERROR;
		error_hook("Cache in error state \r\n");
		return;
	}

	/* De-assert clamps */
	val = NV_DRF_DEF(PMC_IMPL, PART_AOPG_CACHE_CLAMP_CONTROL, CLAMP, OFF);
	writel(val, NV_ADDRESS_MAP_PMC_IMPL_BASE +
				PMC_IMPL_PART_AOPG_CACHE_CLAMP_CONTROL_0);
	udelay(2);
}

enum pm_ram_pg_status get_cache_power_status(void)
{
	return cache_pg_state;
}

void pm_suspend_sc7(void)
{
}

void pm_resume_sc7(void)
{
	clk_init_hw();
}

void pm_hw_init(void)
{
}
