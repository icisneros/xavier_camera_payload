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

#ifndef INCLUDE_TKE_TEGRA_H
#define INCLUDE_TKE_TEGRA_H

#include <stdbool.h>
#include <stdint.h>

struct tegra_tke_id;
typedef void (*tegra_tke_timer_callback)(void *data);

/* Timer clock sources */
#define TEGRA_TKE_CLK_SRC_USECCNT	0U
#define TEGRA_TKE_CLK_SRC_OSCCNT	1U
#define TEGRA_TKE_CLK_SRC_TSC_BIT0	2U
#define TEGRA_TKE_CLK_SRC_TSC_BIT12	3U

/** Maximum divisor for tke timers */
#define TEGRA_TKE_MAX_TIMER 0x20000000UL

/**
 * Initialize TSC and set up TSC to nanosecond conversion.
 */
void tegra_tsc_init(void);

void tegra_tke_timer_writel(const struct tegra_tke_id *id, uint32_t val, uint32_t reg);

/**
 * Set up and start a Tegra timer.
 *
 * If the @a callback is non-NULL, the interrupt is enabled, too.
 *
 * Timer has a PCV counter which starts from one minus the divisor and
 * counts downwards. The interrupt or timer output is triggered when
 * the PCV is 0 and a clock source tick occurs.
 *
 * @param id          The context id of the timer block.
 * @param clk_src_sel Clock source used to drive the timer.
 * @param periodic    If true, start timer in periodic mode.
 * @param divisor     The period of timer in clock source ticks.
 * @param callback    The timer expiration callback invoked in irq context.
 * @param data        The parameter to the callback.
 *
 * The @a divisor must be TEGRA_TKE_MAX_TIMER or less.
 */
void tegra_tke_set_up_timer(const struct tegra_tke_id *id, uint32_t clk_src_sel,
			bool periodic, uint32_t divisor,
			tegra_tke_timer_callback callback, void *data);
/**
 * Stop a Tegra timer.
 *
 * @param id          The context id of the timer block.
 */
void tegra_tke_stop_timer(const struct tegra_tke_id *id);

/**
 * Get current period counter value of a Tegra timer.
 *
 * @param id          The context id of the timer block.
 *
 * @return Period counter value.
 */
uint32_t tegra_tke_get_pcv(const struct tegra_tke_id *id);

/**
 * Get current ATR Target value of a Tegra timer.
 *
 * @param id          The context id of the timer block.
 *
 * @return ATR Target Value.
 */
uint32_t tegra_tke_get_atr(const struct tegra_tke_id *id);

/**
 * Enable the timer interrupt.
 *
 * @param id          The context id of the timer block.
 */
void tegra_tke_enable_timer_irq(const struct tegra_tke_id *id);

/**
 * Disable the timer interrupt.
 *
 * @param id          The context id of the timer block.
 */
void tegra_tke_disable_timer_irq(const struct tegra_tke_id *id);

/**
 * Clear pending timer interrupt.
 *
 * @param id          The context id of the timer block.
 */
void tegra_tke_clear_timer_irq(const struct tegra_tke_id *id);

/**
 * Timer interrupt function.
 *
 * @param id        The context id of the timer block.
 */
void tegra_tke_irq(void *id);

/**
 * Get the 64-bit TSC timer from @a top_tke_base block.
 */
uint64_t tegra_tke_get_tsc64(void);

/**
 * Get the TSC timer from @a top_tke_base block.
 *
 * @param tsc_hi[out] The upper 32 bits of TSC counter.
 * @param tsc_lo[out] The lower 32 bits of TSC counter.
 */
void tegra_tke_get_tsc(uint32_t *tsc_hi, uint32_t *tsc_lo);

/**
 * Get the number of microseconds elapsed since given TCS timestamp.
 *
 * @param prev_tsc_hi[in] The upper 32 bits of TSC timestamp.
 * @param prev_tsc_lo[in] The lower 32 bits of TSC timestamp.
 *
 * @note Requires a tegra_tsc_init() init call.
 */
uint64_t tegra_tke_get_elapsed_usec(uint32_t prev_tsc_hi, uint32_t prev_tsc_low);

/**
 * Get the usec counter value from @a top_tke_base.
 */
uint32_t tegra_tke_get_usec(void);

/**
 * Get the osc counter value from @a top_tke_base.
 */
uint32_t tegra_tke_get_osc(void);

/**
 * Get the low 32 bits of TSC counter value from @a top_tke_base.
 */
uint32_t tegra_tke_get_tsc32(void);

/**
 * Get the current TSC counter value as nanoseconds from @a top_tke_base.
 *
 * @note Requires a tegra_tsc_init() init call.
 */
uint64_t tegra_tke_get_tsc_ns(void);

/**
 * Converta a TSC timestap to nanoseconds.
 *
 * @note Requires a tegra_tsc_init() init call.
 */
uint64_t tegra_tke_tsc_to_ns(uint64_t tsc);

/**
 * Set up FreeRTOS tick timer.
 *
 * @param id       The context id of the timer block.
 * @param clk_src  Clock source used to drive the timer.
 * @param divisor  The period of timer in clock source ticks.
 */
void tegra_tke_set_up_tick(const struct tegra_tke_id *id, uint32_t clk_src,
				uint32_t divisor);

#endif	/* INCLUDE_TKE_TEGRA_H */
