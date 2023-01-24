/*
 * Copyright (c) 2018 NVIDIA CORPORATION.  All rights reserved.
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

#ifndef INCLUDED_TEGRA_SAFETY_LIC_H
#define INCLUDED_TEGRA_SAFETY_LIC_H

#include <stdint.h>

#define TEGRA_SAFETY_LIC_MAX_CHANNELS		2U
#define TEGRA_SAFETY_LIC_MAX_SLICES		11U

#define TEGRA_SAFETY_LIC_LOGICAL_CHANNELS	(2U * TEGRA_SAFETY_LIC_MAX_CHANNELS)

struct tegra_safety_lic;

void tegra_safety_lic_init(struct tegra_safety_lic *lic);
void tegra_safety_lic_enable(struct tegra_safety_lic *lic, uint32_t irq);
void tegra_safety_lic_disable(struct tegra_safety_lic *lic, uint32_t irq);
uint32_t tegra_safety_lic_get_corrected_count(const struct tegra_safety_lic *lic);
void tegra_safety_lic_clear_corrected_count(struct tegra_safety_lic *lic);
uint32_t tegra_safety_lic_get_uncorrected_count(const struct tegra_safety_lic *lic);
void tegra_safety_lic_clear_uncorrected_count(struct tegra_safety_lic *lic);
uint32_t tegra_safety_lic_get_swerror_count(const struct tegra_safety_lic *lic);
void tegra_safety_lic_clear_swerror_count(struct tegra_safety_lic *lic);
void tegra_safety_lic_check_config(struct tegra_safety_lic *lic);
void tegra_safety_lic_restore_config(struct tegra_safety_lic *lic);

#endif	/* INCLUDED_TEGRA_SAFETY_LIC_H */
