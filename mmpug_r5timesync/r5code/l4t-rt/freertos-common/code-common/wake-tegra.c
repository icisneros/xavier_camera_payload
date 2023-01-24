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

#include <arwake.h>
#include <nvrm_drf.h>

#include <macros.h>
#include <reg-access.h>
#include <wake-tegra.h>

/*
 * Current arwake.h is out of date and need to correct below registers
 * Once arwake.h is updated, below registers define should be removed
 */
#undef WAKE_AOWAKE_TIER0_ROUTING_31_0_0
#define WAKE_AOWAKE_TIER0_ROUTING_31_0_0	0x4B4
#undef WAKE_AOWAKE_TIER1_ROUTING_31_0_0
#define WAKE_AOWAKE_TIER1_ROUTING_31_0_0	0x4C0
#undef WAKE_AOWAKE_DET_EN_0
#define WAKE_AOWAKE_DET_EN_0			0x4F0

#define TIER_SIZE \
	(WAKE_AOWAKE_TIER1_ROUTING_31_0_0 - WAKE_AOWAKE_TIER0_ROUTING_31_0_0)

static inline uint32_t tegra_wake_readl(struct tegra_wake_id *id, uint32_t reg)
{
	uint32_t wake_base = (uint32_t)id;

	return readl(wake_base + reg);
}

static inline void tegra_wake_writel(struct tegra_wake_id *id, uint32_t val, uint32_t reg)
{
	uint32_t wake_base = (uint32_t)id;

	writel(val, wake_base + reg);
}

void tegra_wake_route_event(struct tegra_wake_id *id, uint32_t wake_event,
				  uint32_t tier, bool sel)
{
	uint32_t val, reg;

	reg = WAKE_AOWAKE_TIER0_ROUTING_31_0_0 + (tier * TIER_SIZE);
	reg += (wake_event / 32) * 4;
	val = tegra_wake_readl(id, reg);
	if (sel)
		val |= BIT(wake_event % 32);
	else
		val &= ~BIT(wake_event % 32);
	tegra_wake_writel(id, val, reg);
}

void tegra_wake_enable_event(struct tegra_wake_id *id, uint32_t wake_event)
{
	uint32_t reg;

	reg = WAKE_AOWAKE_MASK_W_0 + (wake_event * 4);
	tegra_wake_writel(id, WAKE_AOWAKE_MASK_W_0_MASK_UNMASK, reg);
}

void tegra_wake_disable_event(struct tegra_wake_id *id, uint32_t wake_event)
{
	uint32_t reg;

	reg = WAKE_AOWAKE_MASK_W_0 + (wake_event * 4);
	tegra_wake_writel(id, WAKE_AOWAKE_MASK_W_0_MASK_MASK, reg);
}

void tegra_wake_enable(struct tegra_wake_id *id)
{
	tegra_wake_writel(id, WAKE_AOWAKE_DET_EN_0_DET_ENABLE_TRUE,
			  WAKE_AOWAKE_DET_EN_0);
}

void tegra_wake_disable(struct tegra_wake_id *id)
{
	tegra_wake_writel(id, WAKE_AOWAKE_DET_EN_0_DET_ENABLE_FALSE,
			  WAKE_AOWAKE_DET_EN_0);
}

void tegra_wake_clear_irq(struct tegra_wake_id *id, uint32_t wake_event)
{
	tegra_wake_writel(id, 1, WAKE_AOWAKE_STATUS_W_0 + (wake_event * 4));
}

void tegra_wake_trigger_wake_event(struct tegra_wake_id *id)
{
	tegra_wake_writel(id, 1, WAKE_AOWAKE_SW_WAKE_TIER0_TRIGGER_0);
}
