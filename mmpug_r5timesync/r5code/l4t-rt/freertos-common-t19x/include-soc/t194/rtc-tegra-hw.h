/*
 * Copyright (c) 2016, NVIDIA CORPORATION. All rights reserved.
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

#ifndef _RTC_TEGRA_HW_H_
#define _RTC_TEGRA_HW_H_

#include <address_map_new.h>

enum tegra_rtc_alarm {
	TEGRA_RTC_SEC_ALARM0,
	TEGRA_RTC_SEC_ALARM1,
	TEGRA_RTC_MSEC_ALARM,
	TEGRA_RTC_SEC_COUNTDOWN_ALARM,
	TEGRA_RTC_MSEC_COUNTDOWN_ALARM,
};

/*
 * We cheat and use the HW register address instead of the address of a config
 * structure as the RTC ID for now. We still need a dummy definition of the
 * struct to do that though; this is it.
 */
struct tegra_rtc_id {
	uint32_t dummy;
};

/*
 * We intend: &tegra_rtc_id == NV_ADDRESS_MAP_RTC_BASE. This avoids
 * having to allocate a struct to hold the value NV_ADDRESS_MAP_RTC_BASE. If
 * we ever need to store more configuration than just the register address,
 * this can be converted to a struct at that time.
 */
#define tegra_rtc_id_rtc (*(struct tegra_rtc_id *)NV_ADDRESS_MAP_RTC_BASE)

#endif
