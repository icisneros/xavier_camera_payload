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

#ifndef _RTC_TEGRA_H_
#define _RTC_TEGRA_H_

#include <stdint.h>

struct tegra_rtc_id;

/*
 * Get seconds elapsed
 *
 * Parameters:
 * id:		The ID structure for the RTC module.
 *
 * Returns:
 * value of second counter
 */
uint32_t tegra_rtc_get_sec(struct tegra_rtc_id *id);

/*
 * Get milli-seconds elapsed
 *
 * Parameters:
 * id:		The ID structure for the RTC module.
 *
 * Returns:
 * value of (second counter * 1000) + milli-second counter
 */
uint64_t tegra_rtc_get_ms(struct tegra_rtc_id *id);

/*
 * Enable RTC ALARM IRQ
 *
 * Parameters:
 * id:		The ID structure for the RTC module.
 * alarm:	The alarm irq to be enabled
 */
void tegra_rtc_enable_alarm_irq(struct tegra_rtc_id *id, uint32_t alarm);

/*
 * Disable RTC ALARM IRQ
 *
 * Parameters:
 * id:		The ID structure for the RTC module.
 * alarm:	The alarm irq to be disabled
 */
void tegra_rtc_disable_alarm_irq(struct tegra_rtc_id *id, uint32_t alarm);

/*
 * Set up milli-second countdown alarm
 *
 * Parameters:
 * id:		The ID structure for the RTC module.
 * ms:		number of milli-seconds to countdown
 */
void tegra_rtc_set_up_countdown_msec_alarm(struct tegra_rtc_id *id, uint32_t ms);

/*
 * Clear RTC interrupt status
 *
 * Parameters:
 * id:		The ID structure for the RTC module.
 */
void tegra_rtc_clear_irq(struct tegra_rtc_id *id);

#endif
