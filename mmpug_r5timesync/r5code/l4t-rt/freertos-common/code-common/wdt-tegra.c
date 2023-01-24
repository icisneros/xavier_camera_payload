/*
 * Copyright (c) 2015-2017 NVIDIA CORPORATION.  All rights reserved.
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

#include <inttypes.h>
#include <stdint.h>
#include <stddef.h>

#include <FreeRTOS.h>
#include <task.h>

#include <artke_top.h>
#include <nvrm_drf.h>

#include <err-hook.h>
#include <irqs.h>
#include <macros.h>
#include <reg-access.h>
#include <wdt-tegra-priv.h>

#define TEGRA_WDT_CONFIG_0		(TKE_TOP_WDT0_WDTCR_0 - BASE_ADDRESS_TKE_TOP_WDT0)
#define TEGRA_WDT_STATUS_0		(TKE_TOP_WDT0_WDTSR_0 - BASE_ADDRESS_TKE_TOP_WDT0)
#define TEGRA_WDT_COMMAND_0		(TKE_TOP_WDT0_WDTCMDR_0 - BASE_ADDRESS_TKE_TOP_WDT0)
#define TEGRA_WDT_UNLOCK_PATTERN_0	(TKE_TOP_WDT0_WDTUR_0 - BASE_ADDRESS_TKE_TOP_WDT0)
#define TEGRA_WDT_UNLOCK_PATTERN_VALUE	0xC45AU
#define TEGRA_WDT_UNLOCK_CRC32		0xEDB88320U

static inline struct tegra_wdt *tegra_wdt_from_id(const struct tegra_wdt_id *id)
{
	return CONTAINER_OF(id, struct tegra_wdt, id);
}

static inline uint32_t tegra_wdt_readl(const struct tegra_wdt_id *id,
				uint32_t reg)
{
	return readl(id->base_addr + reg);
}

static inline void tegra_wdt_writel(const struct tegra_wdt_id *id,
				uint32_t val, uint32_t reg)
{
	writel(val, id->base_addr + reg);
}

static void tegra_wdt_unlock(const struct tegra_wdt_id *id)
{
	const struct tegra_wdt *wdt_dev = tegra_wdt_from_id(id);
	uint32_t val;

	if (wdt_dev->challenge_response) {
		/*
		 * The Challenge is a LFSR value, present in the Unlock
		 * register.
		 *
		 * The Response is the next value of the LFSR, that needs to
		 * be written in the Unlock register.
		 *
		 * The LFSR uses the classical CRC-32 polynomial operating
		 * in shift right (reversed) direction, that is the next
		 * value is calculated as
		 *
		 * LFSR = (LFSR >> 1) ^ ((LFSR & 0x1) ? 0xEDB88320 : 0)
		 */
		val = tegra_wdt_readl(id, TEGRA_WDT_UNLOCK_PATTERN_0);
		val = (val >> 1) ^ ((val & 1U) ? TEGRA_WDT_UNLOCK_CRC32 : 0);
	} else {
		val = TEGRA_WDT_UNLOCK_PATTERN_VALUE;
	}

	tegra_wdt_writel(id, val, TEGRA_WDT_UNLOCK_PATTERN_0);
}

static void _tegra_wdt_ack(const struct tegra_wdt_id *id)
{
	tegra_wdt_unlock(id);
	tegra_wdt_start(id);
}

void tegra_wdt_start(const struct tegra_wdt_id *id)
{
	const struct tegra_wdt *wdt_dev = tegra_wdt_from_id(id);
	uint32_t val;

	/*
	 * When written to 1b, enable the counter operation, load the counter
	 * with Period and starts downcounting, resets the expiration count to
	 * 0 and clears all status flags.
	 */
	val = NV_DRF_NUM(TKE_TOP, WDT0_WDTCMDR, StartCounter, 1);
	tegra_wdt_writel(id, val, TEGRA_WDT_COMMAND_0);

	if (wdt_dev->irq_callback) {
		irq_enable(id->irq);
	}
	if (wdt_dev->fiq_callback) {
		irq_enable(id->fiq);
	}
}

void tegra_wdt_stop(const struct tegra_wdt_id *id)
{
	const struct tegra_wdt *wdt_dev = tegra_wdt_from_id(id);
	uint32_t val;

	/*
	 * Only working if the unlock register (WDTUR{w}) has been
	 * programmed before with the correct pattern. Writing to the command
	 * register always clears the unlock register.
	 * If written DisableCounter to 1b, while WDT counter is enabled and the
	 * unlock register contains the unlock pattern, the watchdog transitions
	 * back to disabled.
	 */

	if (!in_interrupt())
		taskENTER_CRITICAL();

	tegra_wdt_unlock(id);

	val = NV_DRF_NUM(TKE_TOP, WDT0_WDTCMDR, DisableCounter, 1);
	tegra_wdt_writel(id, val, TEGRA_WDT_COMMAND_0);

	if (wdt_dev->irq_callback) {
		irq_disable(id->irq);
	}
	if (wdt_dev->fiq_callback) {
		irq_disable(id->fiq);
	}

	if (!in_interrupt())
		taskEXIT_CRITICAL();
}

void tegra_wdt_stop_from_isr(const struct tegra_wdt_id *id)
	__attribute__((alias("tegra_wdt_stop")));

void tegra_wdt_irq(void *data)
{
	const struct tegra_wdt_id *id = data;
	const struct tegra_wdt *wdt_dev = tegra_wdt_from_id(id);
	uint32_t status;

	status = tegra_wdt_read_status(id);

	if (wdt_dev->irq_callback) {
		if (wdt_dev->irq_ack_by_callback) {
			irq_disable(id->irq);
		} else {
			_tegra_wdt_ack(id);
		}
		wdt_dev->irq_callback(status, wdt_dev->irq_data);
	}
}

void tegra_wdt_fiq(void *data)
{
	const struct tegra_wdt_id *id = data;
	const struct tegra_wdt *wdt_dev = tegra_wdt_from_id(id);
	uint32_t status;

	status = tegra_wdt_read_status(id);

	if (wdt_dev->fiq_callback) {
		if (wdt_dev->fiq_ack_by_callback) {
			irq_disable(id->fiq);
		} else {
			_tegra_wdt_ack(id);
		}
		wdt_dev->fiq_callback(status, wdt_dev->fiq_data);
	}
}

uint32_t tegra_wdt_read_status(const struct tegra_wdt_id *id)
{
	return tegra_wdt_readl(id, TEGRA_WDT_STATUS_0);
}

void tegra_wdt_ack(const struct tegra_wdt_id *id)
{
	const struct tegra_wdt *wdt_dev = tegra_wdt_from_id(id);

	if (!in_interrupt())
		taskENTER_CRITICAL();

	_tegra_wdt_ack(id);

	if (wdt_dev->irq_ack_by_callback) {
		irq_enable(id->irq);
	}
	if (wdt_dev->fiq_ack_by_callback) {
		irq_enable(id->fiq);
	}

	if (!in_interrupt())
		taskEXIT_CRITICAL();
}

int tegra_wdt_setup(const struct tegra_wdt_id *id, const struct tegra_wdt_conf *conf)
{
	struct tegra_wdt *wdt_dev = tegra_wdt_from_id(id);
	uint32_t val;

	if (conf->tmrsrc & ~TKE_TOP_WDT0_WDTCR_0_TimerSource_DEFAULT_MASK)
		return 1;

	if (conf->err_threshold & ~TKE_TOP_WDT0_WDTCR_0_ErrorThreshold_DEFAULT_MASK)
		return 1;

	if (conf->irq_en && (!conf->irq_callback))
		return 1;

	if (conf->fiq_en && (!conf->fiq_callback))
		return 1;

	wdt_dev->irq_ack_by_callback = conf->irq_en && conf->irq_ack_by_callback;
	wdt_dev->irq_callback = conf->irq_en ? conf->irq_callback : NULL;
	wdt_dev->irq_data = conf->irq_data;
	wdt_dev->fiq_ack_by_callback = conf->fiq_en && conf->fiq_ack_by_callback;
	wdt_dev->fiq_callback = conf->fiq_en ? conf->fiq_callback : NULL;
	wdt_dev->fiq_data = conf->fiq_data;

	val =
		NV_DRF_NUM(TKE_TOP, WDT0_WDTCR, TimerSource, conf->tmrsrc) |
		NV_DRF_NUM(TKE_TOP, WDT0_WDTCR, Period, conf->period) |
		NV_DRF_NUM(TKE_TOP, WDT0_WDTCR, LocalInterruptEnable, conf->irq_en) |
		NV_DRF_NUM(TKE_TOP, WDT0_WDTCR, LocalFIQEnable, conf->fiq_en) |
		NV_DRF_NUM(TKE_TOP, WDT0_WDTCR, RemoteInterruptEnable, conf->remoteIrq_en) |
		NV_DRF_NUM(TKE_TOP, WDT0_WDTCR, SystemDebugResetEnable, conf->sys_dbg_rst_en) |
		NV_DRF_NUM(TKE_TOP, WDT0_WDTCR, SystemPOResetEnable, conf->sys_por_rst_en) |
		NV_DRF_NUM(TKE_TOP, WDT0_WDTCR, TscReferenceEnable, conf->tsc_ref_en) |
		NV_DRF_NUM(TKE_TOP, WDT0_WDTCR, ErrorThreshold, conf->err_threshold);

#ifndef HW_BUG_1776680
	tegra_wdt_writel(id, val, TEGRA_WDT_CONFIG_0);
#else
	{
		uint32_t cfg0 = tegra_wdt_readl(id, TEGRA_WDT_CONFIG_0);
		if (cfg0 != val) {
			warning_hookf("wdtcr@%p has 0x%08"PRIx32", "
				"trying to set 0x%08"PRIx32,
				(void *)(id->base_addr + TEGRA_WDT_CONFIG_0),
				cfg0, val);
		}
		val = cfg0;
	}
#endif

	if (val & NV_DRF_NUM(TKE_TOP, WDT0_WDTCR, ChallengeResponseEnable, 1)) {
		wdt_dev->challenge_response = true;
	} else {
		wdt_dev->challenge_response = false;
	}

	if (conf->irq_en) {
		irq_set_handler(id->irq, tegra_wdt_irq, (void *)id);
	}

	if (conf->fiq_en) {
		irq_set_handler(id->fiq, tegra_wdt_fiq, (void *)id);
	}

	return 0;
}
