/*
 * Copyright (c) 2015, NVIDIA CORPORATION.  All rights reserved.
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

#include <arrtc.h>
#include <nvrm_drf.h>

#include <delay.h>
#include <err-hook.h>
#include <macros.h>
#include <reg-access.h>
#include <rtc-tegra.h>

#define RTC_BUSY_CHK_TIMEOUT	200

static inline uint32_t tegra_rtc_readl(struct tegra_rtc_id *id, uint32_t reg)
{
	uint32_t rtc_base = (uint32_t)id;

	return readl(rtc_base + reg);
}

static inline void tegra_rtc_writel(struct tegra_rtc_id *id, uint32_t val,
				    uint32_t reg)
{
	uint32_t rtc_base = (uint32_t)id;
	uint64_t tstart;

	tstart = get_time_ticks();
	while (tegra_rtc_readl(id, APBDEV_RTC_BUSY_0)) {
		if (get_time_delta_us(tstart) > RTC_BUSY_CHK_TIMEOUT) {
			error_hook("rtc busy");
			break;
		}
	}

	writel(val, rtc_base + reg);
}

uint32_t tegra_rtc_get_sec(struct tegra_rtc_id *id)
{
	return tegra_rtc_readl(id, APBDEV_RTC_SECONDS_0);
}

uint64_t tegra_rtc_get_ms(struct tegra_rtc_id *id)
{
	uint32_t ms, sec;

	ms = tegra_rtc_readl(id, APBDEV_RTC_MILLI_SECONDS_0);
	sec = tegra_rtc_readl(id, APBDEV_RTC_SHADOW_SECONDS_0);

	return (((uint64_t)sec) * 1000) + ms;
}

void tegra_rtc_enable_alarm_irq(struct tegra_rtc_id *id, uint32_t alarm)
{
	uint32_t val;

	val = tegra_rtc_readl(id, APBDEV_RTC_INTR_MASK_0);
	val |= BIT(alarm);
	tegra_rtc_writel(id, val, APBDEV_RTC_INTR_MASK_0);
}

void tegra_rtc_disable_alarm_irq(struct tegra_rtc_id *id, uint32_t alarm)
{
	uint32_t val;

	val = tegra_rtc_readl(id, APBDEV_RTC_INTR_MASK_0);
	val &= ~BIT(alarm);
	tegra_rtc_writel(id, val, APBDEV_RTC_INTR_MASK_0);
}

void tegra_rtc_set_up_countdown_msec_alarm(struct tegra_rtc_id *id, uint32_t ms)
{
	uint32_t val;

	val =
		NV_DRF_NUM(APBDEV_RTC, MILLI_SECONDS_COUNTDOWN_ALARM, VALUE, ms) |
		NV_DRF_NUM(APBDEV_RTC, MILLI_SECONDS_COUNTDOWN_ALARM, ENABLE, 1);
	tegra_rtc_writel(id, val, APBDEV_RTC_MILLI_SECONDS_COUNTDOWN_ALARM_0);
}

void tegra_rtc_clear_irq(struct tegra_rtc_id *id)
{
	uint32_t val;

	val = tegra_rtc_readl(id, APBDEV_RTC_INTR_STATUS_0);
	tegra_rtc_writel(id, val, APBDEV_RTC_INTR_STATUS_0);
}
