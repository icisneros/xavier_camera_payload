/*
 * Copyright (c) 2018, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef INCLUDED_TEGRA_EC_PRIV_H
#define INCLUDED_TEGRA_EC_PRIV_H

#include <stdint.h>
#include <stdbool.h>

/*
 * Though Error collator HW supports max of 16 slices and 512 Errors
 *  SCE Error collators have only max 3 slices and max 96 errors
*/

#define MAX_EC_SLICES				3U
#define MAX_EC_SEC_THRESHOLD			0xFFU
#define MAX_EC_ERRORS				96U
#define EC_ERRORS_PER_SLICE			32
/* Reference Error collator IAS */

/* Global Error collator registers */
#define EC_FEATURE_OFFSET			0x00U
#define EC_SWRESET_OFFSET			0x04U
#define EC_MISSIONERR_TYPE_OFFSET		0x08U
#define EC_CURRENT_COUNTER_VALUE_OFFSET		0x0CU
#define EC_MISSIONERR_USERVALUE_OFFSET		0x10U
#define EC_MISSIONERR_INDEX_OFFSET		0x14U
#define EC_CORRECTABLE_THRESHOLD_OFFSET		0x18U
#define EC_MISSIONERR_INJECT_UNLOCK_OFFSET	0x1CU
/* offset 0x20 to 0x2C reserved */

/* Per slice Error collator registers */
#define EC_MISSIONERR_ENABLE_OFFSET(X)		((0x30*((X)+1))+0x00)
#define EC_MISSIONERR_FORCE_OFFSET(X)		((0x30*((X)+1))+0x04)
#define EC_MISSIONERR_STATUS_OFFSET(X)		((0x30*((X)+1))+0x08)
#define EC_MISSIONERR_INJECT_OFFSET(X)		((0x30*((X)+1))+0x0C)
#define EC_LATENTERR_ENABLE_OFFSET(X)		((0x30*((X)+1))+0x10)
#define EC_LATENTERR_FORCE_OFFSET(X)		((0x30*((X)+1))+0x14)
#define EC_LATENTERR_STATUS_OFFSET(X)		((0x30*((X)+1))+0x18)
#define EC_COUNTER_RELOAD_OFFSET(X)		((0x30*((X)+1))+0x20)

struct tegra_ec_id {
	const char *devname;
	uint32_t base;
	uint32_t fatal_err_index;
	uint32_t corr_err_index;
	uint32_t error_inject_mask[MAX_EC_SLICES];
	uint32_t user_value_support;
};

struct tegra_ec
{
	/*
	 * The driver relies on this being the first field, since it casts
	 * pointers between structs tegra_ec_id and tegra_ec.
	 */
	const struct tegra_ec_id conf;
	uint32_t total_errors;
	uint32_t total_slices;
	bool error_inject_lock;
	uint32_t sw_reset_count;
	uint32_t sec_threshold;
	uint32_t mission_status;
	uint32_t latent_status;
	struct tegra_ec_handler ec_handlers[MAX_EC_ERRORS];
};

#endif
