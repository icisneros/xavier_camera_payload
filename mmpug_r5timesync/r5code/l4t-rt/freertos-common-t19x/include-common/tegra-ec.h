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

#ifndef INCLUDED_TEGRA_EC_H
#define INCLUDED_TEGRA_EC_H

#include <stdint.h>

struct tegra_ec_id;

typedef void (*ec_error_handler)(void *arg);

struct tegra_ec_handler {
	void *data;
	ec_error_handler mission_handler;
	ec_error_handler latent_handler;
};

struct tegra_ec_error {
	uint32_t type;
	uint32_t counter;
	uint32_t user_value;
};

int tegra_ec_init(const struct tegra_ec_id *id);
void tegra_ec_deinit(const struct tegra_ec_id *id);
int tegra_ec_sw_reset(const struct tegra_ec_id *id);
int tegra_ec_read_error(const struct tegra_ec_id *id,
			uint32_t index,
			struct tegra_ec_error *error);
int tegra_ec_err_inject_lock(const struct tegra_ec_id *id);
int tegra_ec_err_inject_unlock(const struct tegra_ec_id *id);
int tegra_ec_enable_mission_error(const struct tegra_ec_id *id, uint32_t index);
int tegra_ec_disable_mission_error(const struct tegra_ec_id *id, uint32_t index);
int tegra_ec_force_mission_error(const struct tegra_ec_id *id, uint32_t index);
int tegra_ec_clear_mission_error(const struct tegra_ec_id *id, uint32_t index);
int tegra_ec_set_inject_mission_error(const struct tegra_ec_id *id, uint32_t index);
int tegra_ec_clear_inject_mission_error(const struct tegra_ec_id *id, uint32_t index);
int tegra_ec_enable_latent_error(const struct tegra_ec_id *id, uint32_t index);
int tegra_ec_disable_latent_error(const struct tegra_ec_id *id, uint32_t index);
int tegra_ec_force_latent_error(const struct tegra_ec_id *id, uint32_t index);
int tegra_ec_clear_latent_error(const struct tegra_ec_id *id, uint32_t index);
int tegra_ec_counter_reload(const struct tegra_ec_id *id, uint32_t index);
void tegra_ec_error_handler(void *data);
int tegra_ec_register_handler(
			const struct tegra_ec_id *id,
			uint32_t index,
			const struct tegra_ec_handler *handler);
int tegra_ec_get_mission_error_status(
			const struct tegra_ec_id *id,
			uint32_t index,
			uint32_t *status);
int tegra_ec_get_latent_error_status(
			const struct tegra_ec_id *id,
			uint32_t index,
			uint32_t *status);
int tegra_ec_set_sec_threshold(const struct tegra_ec_id *id, uint32_t threshold);
#endif
