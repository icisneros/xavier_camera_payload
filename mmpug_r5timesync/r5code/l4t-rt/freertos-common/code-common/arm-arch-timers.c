/*
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
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

#include <FreeRTOS.h>
#include <task.h>

#include <arm-arch-timers.h>
#include <delay.h>
#include <macros.h>

#define CNTV_CTL_ENABLE		BIT(0)
#define CNTV_CTL_IMASK		BIT(1)
#define CNTV_CTL_ISTATUS	BIT(2)

static uint32_t timer_event_inc;
static uint64_t timer_next_event;

static inline uint32_t timer_read_cntfrq(void)
{
	uint32_t val;

	__asm__ __volatile__("mrc p15, 0, %0, c14, c0, 0\n\t" : "=r" (val));

	return val;
}

static inline uint64_t timer_read_cntvct(void)
{
	uint32_t val_l, val_h;

	__asm__ __volatile__("mrrc p15, 1, %0, %1, c14\n\t"
		: "=r" (val_l), "=r" (val_h));

	return (((uint64_t)val_h) << 32) | val_l;
}

static inline void timer_write_cntv_ctl(uint32_t val)
{
	__asm__ __volatile__("mcr p15, 0, %0, c14, c3, 1\n\t" : : "r" (val));
}

static inline void timer_write_cntv_cval(uint64_t val)
{
	__asm__ __volatile__("mcrr p15, 3, %0, %1, c14\n\t"
		: : "r" (val & 0xffffffffULL), "r"  (val >> 32));
}

void arm_arch_timers_init(void)
{
	timer_event_inc = timer_read_cntfrq() / configTICK_RATE_HZ;
	timer_next_event = timer_read_cntvct() + timer_event_inc;

	timer_write_cntv_cval(timer_next_event);
	timer_write_cntv_ctl(CNTV_CTL_ENABLE);
}

void arm_arch_timers_on_expiry(void)
{
	timer_next_event += timer_event_inc;
	timer_write_cntv_cval(timer_next_event);
}

void udelay(uint32_t delay_us)
{
	uint64_t time = timer_read_cntvct();
	uint64_t delay_ticks = (((uint64_t)delay_us) * timer_read_cntfrq()) / 1000000;
	uint64_t expiry = time + delay_ticks;

	/* Ignore rollover; 64 bits at 38.4MHz is ~15000 years uptime */
	while (timer_read_cntvct() < expiry)
		taskYIELD();
}

uint64_t get_time_ticks(void)
{
	return timer_read_cntvct();
}

uint32_t get_time_delta_us(uint64_t ticks_start)
{
	uint64_t delta = timer_read_cntvct() - ticks_start;
	return (delta * 1000) / (timer_read_cntfrq() / 1000);
}
