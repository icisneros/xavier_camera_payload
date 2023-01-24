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

#include <FreeRTOS.h>
#include <task.h>

#include <stdint.h>

#include <address_map_new.h>
#include <artke_top.h>

#include <delay.h>
#include <bitops.h>
#include <irqs.h>
#include <macros.h>
#include <reg-access.h>
#include <tke-tegra-priv.h>
#include <tke-tegra.h>

#define TKE_TIMER_TMRCR_0		0x0
#define TKE_TIMER_TMRCR_0_PTV		0x1fffffffU
#define TKE_TIMER_TMRCR_0_PER		BIT(30)
#define TKE_TIMER_TMRCR_0_EN		BIT(31)

#define TKE_TIMER_TMRSR_0		0x4
#define TKE_TIMER_TMRSR_0_INTR_CLR	BIT(30)
#define TKE_TIMER_TMRSR_0_PCV		0x1fffffffU

#define TKE_TIMER_TMRATR_0		0xc
#define TKE_TIMER_TMRATR_0_ATR		0x3fffffffU

#define TKE_TIMER_TMRCSSR_0		0x8

#define TSC_MTSCANNR_0			0x8
#define TSC_MTSCANDR_0			0xc

#define TSC_MTSCANNR_0_M0_SHIFT		16
#define TSC_MTSCANNR_0_R0_SHIFT		0
#define TSC_MTSCANDR_0_D0_SHIFT		0
#define TSC_MTSCANNR_0_M0_MASK		(0xfff << TSC_MTSCANNR_0_M0_SHIFT)
#define TSC_MTSCANNR_0_R0_MASK		(0xfff << TSC_MTSCANNR_0_R0_SHIFT)
#define TSC_MTSCANDR_0_D0_MASK		(0xfff << TSC_MTSCANDR_0_D0_SHIFT)

#define TSC_BASE_RATE			32768UL

static uint32_t cntfid0, cntperiod;

static inline uint32_t tegra_tke_timer_readl(const struct tegra_tke_id *id, uint32_t reg)
{
	return readl(id->base_addr + reg);
}

 void tegra_tke_timer_writel(const struct tegra_tke_id *id, uint32_t val, uint32_t reg)
{
	writel(val, id->base_addr + reg);
}

void tegra_tke_set_up_timer(const struct tegra_tke_id *id, uint32_t clk_src_sel,
				bool periodic, uint32_t divisor,
				tegra_tke_timer_callback callback, void *data)
{
	/* struct tegra_tke_id assumed to be first field in tegra_tke */
	struct tegra_tke *tke_dev = (struct tegra_tke *)id;
	uint32_t tmrcr;

	tke_dev->callback = callback;
	tke_dev->data = data;

	/* select timer clock source */
	tegra_tke_timer_writel(id, clk_src_sel, TKE_TIMER_TMRCSSR_0);

	/* Disable timer, set cycle counter */
	tmrcr = (divisor - 1) & TKE_TIMER_TMRCR_0_PTV;
	tegra_tke_timer_writel(id, tmrcr, TKE_TIMER_TMRCR_0);

	/* Clear timer interrupts. */
	tegra_tke_timer_writel(id, TKE_TIMER_TMRSR_0_INTR_CLR, TKE_TIMER_TMRSR_0);

	/* Enable timer and configure timer type. */
	tmrcr |= TKE_TIMER_TMRCR_0_EN;
	if (periodic)
		tmrcr |= TKE_TIMER_TMRCR_0_PER;
	tegra_tke_timer_writel(id, tmrcr, TKE_TIMER_TMRCR_0);

	if (id->irq != TEGRA_TKE_NO_IRQ) {
		irq_set_handler(id->irq, tegra_tke_irq, tke_dev);
		irq_enable(id->irq);
	}
}

void tegra_tke_stop_timer(const struct tegra_tke_id *id)
{
	uint32_t tmrcr;

	if (id->irq != TEGRA_TKE_NO_IRQ) {
		irq_disable(id->irq);
	}

	tmrcr = tegra_tke_timer_readl(id, TKE_TIMER_TMRCR_0);
	tmrcr &= ~TKE_TIMER_TMRCR_0_EN;
	tegra_tke_timer_writel(id, tmrcr, TKE_TIMER_TMRCR_0);
}

uint32_t tegra_tke_get_pcv(const struct tegra_tke_id *id)
{
	uint32_t val;

	val = tegra_tke_timer_readl(id, TKE_TIMER_TMRSR_0);

	return val & TKE_TIMER_TMRSR_0_PCV;
}

uint32_t tegra_tke_get_atr(const struct tegra_tke_id *id)
{
	uint32_t val;

	val = tegra_tke_timer_readl(id, TKE_TIMER_TMRATR_0);

	return val & TKE_TIMER_TMRATR_0_ATR;
}


void tegra_tke_enable_timer_irq(const struct tegra_tke_id *id)
{
	if (id->irq != TEGRA_TKE_NO_IRQ) {
		irq_enable(id->irq);
	}
}

void tegra_tke_disable_timer_irq(const struct tegra_tke_id *id)
{
	if (id->irq != TEGRA_TKE_NO_IRQ) {
		irq_disable(id->irq);
	}
}

void tegra_tke_clear_timer_irq(const struct tegra_tke_id *id)
{
	tegra_tke_timer_writel(id, TKE_TIMER_TMRSR_0_INTR_CLR,
		TKE_TIMER_TMRSR_0);
}

void tegra_tke_irq(void *id)
{
	struct tegra_tke *tke_dev = id;

	tegra_tke_clear_timer_irq(&tke_dev->conf);

	if (tke_dev->callback)
		tke_dev->callback(tke_dev->data);
}

void tegra_tke_get_tsc(uint32_t *tsc_hi, uint32_t *tsc_lo)
{
	uint64_t tsc = tegra_tke_get_tsc64();

	*tsc_lo = (uint32_t)tsc;
	*tsc_hi = (uint32_t)(tsc >> 32);
}

/* TSC timestamp from 31.25 MHz clock (32 ns cycle) */
uint32_t tegra_tke_get_tsc32(void)
{
	return readl(tke_top_base + TKE_TOP_SHARED_TKETSC0_0);
}

uint64_t tegra_tke_get_tsc64(void)
{
	uint32_t hi0, lo, hi;

	/*
	 * Here is how the code to handle rollover of the upper word:
	 * Always read twice the MSW, pattern is read MSW, LSW, MSW.
	 * If the two MSW are OK then it's done, otherwise read LSW a
	 * second time, and keep the second set of both values
	 */
	hi0 = readl(tke_top_base + TKE_TOP_SHARED_TKETSC1_0);
	lo = readl(tke_top_base + TKE_TOP_SHARED_TKETSC0_0);
	hi = readl(tke_top_base + TKE_TOP_SHARED_TKETSC1_0);

	if (hi0 != hi)
		lo = readl(tke_top_base + TKE_TOP_SHARED_TKETSC0_0);

	return hilo_to_64(hi, lo);
}

/* TSC timestamp at nanosecond resolution */
uint64_t tegra_tke_get_tsc_ns(void)
{
	/* TSC (cntfid0) is usually 31.25MHz, cntperiod is 32 ns */
	return tegra_tke_get_tsc64() * cntperiod;
}

/* Convert TSC timestamp to nanosecond resolution */
uint64_t tegra_tke_tsc_to_ns(uint64_t tsc)
{
	/* TSC is 31.25MHz = 32 ns */
	return tsc * cntperiod;
}

uint64_t tegra_tke_get_elapsed_usec(uint32_t prev_tsc_hi, uint32_t prev_tsc_low)
{
	uint64_t result, curr_tsc, prev_tsc;

	curr_tsc = tegra_tke_get_tsc64();
	prev_tsc = hilo_to_64(prev_tsc_hi, prev_tsc_low);

	/* cntfid0 holds the base frequency of the tsc.
	 * This will always be the frequency tsc runs.
	 * There is no plan to change it for tegra
	 */
	result = (curr_tsc - prev_tsc) * cntperiod / 1000;

	return result;
}

uint32_t tegra_tke_get_usec(void)
{
	return readl(tke_top_base + TKE_TOP_SHARED_TKEUSEC_0);
}

uint32_t tegra_tke_get_osc(void)
{
	return readl(tke_top_base + TKE_TOP_SHARED_TKEOSC_0);
}

void tegra_tsc_init(void)
{
	uint32_t m0, r0, d0;
	uint32_t val;

	val = readl(NV_ADDRESS_MAP_TSC_IMPL_BASE + TSC_MTSCANNR_0);

	m0 = (val & TSC_MTSCANNR_0_M0_MASK) >> TSC_MTSCANNR_0_M0_SHIFT;
	r0 = (val & TSC_MTSCANNR_0_R0_MASK) >> TSC_MTSCANNR_0_R0_SHIFT;

	val = readl(NV_ADDRESS_MAP_TSC_IMPL_BASE + TSC_MTSCANDR_0);
	d0 = (val & TSC_MTSCANDR_0_D0_MASK) >> TSC_MTSCANDR_0_D0_SHIFT;

	cntfid0 = (TSC_BASE_RATE / d0) * (d0 * r0 + m0);
	cntperiod = 1000000000U / cntfid0;
}

static void tegra_tke_os_tick(void *data)
{
	FreeRTOS_Tick_Handler();
}

void tegra_tke_set_up_tick(const struct tegra_tke_id *id, uint32_t clk_src,
				uint32_t divisor)
{
	tegra_tke_set_up_timer(id, clk_src, true, divisor, tegra_tke_os_tick,
				NULL);
}
