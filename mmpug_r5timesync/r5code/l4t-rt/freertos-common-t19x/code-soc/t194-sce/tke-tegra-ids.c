/*
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
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

#include <address_map_new.h>

#include <irqs-hw.h>
#include <tke-tegra-priv.h>
#include <tke-tegra-hw.h>

const uint32_t tke_top_base = NV_ADDRESS_MAP_SCE_TKE_BASE;

struct tegra_tke tegra_tke_timer0 = {
	.conf = {
		.devname = "tke0",
		.base_addr = NV_ADDRESS_MAP_SCE_TKE_TMR_0_BASE,
		.irq = NV_SCE_IRQ_TIMER0,
	},
};
extern const struct tegra_tke_id tegra_tke_id_timer0
__attribute__((alias("tegra_tke_timer0")));

struct tegra_tke tegra_tke_timer1 = {
	.conf = {
		.devname = "tke1",
		.base_addr = NV_ADDRESS_MAP_SCE_TKE_TMR_1_BASE,
		.irq = NV_SCE_IRQ_TIMER1,
	},
};
extern const struct tegra_tke_id tegra_tke_id_timer1
__attribute__((alias("tegra_tke_timer1")));

struct tegra_tke tegra_tke_timer2 = {
	.conf = {
		.devname = "tke2",
		.base_addr = NV_ADDRESS_MAP_SCE_TKE_TMR_2_BASE,
		.irq = NV_SCE_IRQ_TIMER2,
	},
};
extern const struct tegra_tke_id tegra_tke_id_timer2
__attribute__((alias("tegra_tke_timer2")));

struct tegra_tke tegra_tke_timer3 = {
	.conf = {
		.devname = "tke3",
		.base_addr = NV_ADDRESS_MAP_SCE_TKE_TMR_3_BASE,
		.irq = NV_SCE_IRQ_TIMER3,
	},
};
extern const struct tegra_tke_id tegra_tke_id_timer3
__attribute__((alias("tegra_tke_timer3")));

static struct tegra_tke tegra_tke_noirq_timer0 __attribute__((unused)) = {
	.conf = {
		.devname = "tke0",
		.base_addr = NV_ADDRESS_MAP_SCE_TKE_TMR_0_BASE,
		.irq = TEGRA_TKE_NO_IRQ,
	},
};
extern const struct tegra_tke_id tegra_tke_id_noirq_timer0
__attribute__((alias("tegra_tke_noirq_timer0")));

static struct tegra_tke tegra_tke_noirq_timer1 __attribute__((unused)) = {
	.conf = {
		.devname = "tke1",
		.base_addr = NV_ADDRESS_MAP_SCE_TKE_TMR_1_BASE,
		.irq = TEGRA_TKE_NO_IRQ,
	},
};
extern const struct tegra_tke_id tegra_tke_id_noirq_timer1
__attribute__((alias("tegra_tke_noirq_timer1")));

static struct tegra_tke tegra_tke_noirq_timer2 __attribute__((unused)) = {
	.conf = {
		.devname = "tke2",
		.base_addr = NV_ADDRESS_MAP_SCE_TKE_TMR_2_BASE,
		.irq = TEGRA_TKE_NO_IRQ,
	},
};
extern const struct tegra_tke_id tegra_tke_id_noirq_timer2
__attribute__((alias("tegra_tke_noirq_timer2")));

static struct tegra_tke tegra_tke_noirq_timer3 __attribute__((unused)) = {
	.conf = {
		.devname = "tke3",
		.base_addr = NV_ADDRESS_MAP_SCE_TKE_TMR_3_BASE,
		.irq = TEGRA_TKE_NO_IRQ,
	},
};
extern const struct tegra_tke_id tegra_tke_id_noirq_timer3
__attribute__((alias("tegra_tke_noirq_timer3")));
