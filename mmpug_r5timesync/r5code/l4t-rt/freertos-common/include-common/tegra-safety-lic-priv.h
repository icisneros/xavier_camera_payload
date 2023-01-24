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

#ifndef INCLUDED_TEGRA_SAFETY_LIC_PRIV_H
#define INCLUDED_TEGRA_SAFETY_LIC_PRIV_H

#include <stdint.h>
#include <stdbool.h>
#include <tegra-safety-lic.h>

#define TEGRA_SAFETY_LIC_MAX_LOGICAL_CHANNELS	(2U * TEGRA_SAFETY_LIC_MAX_CHANNELS)

/* Mark IRQ unused in irq map */
#define TEGRA_SAFETY_LIC_IRQ_UNUSED		0xFFU

/* Mapping of local irqs to LIC channels */
struct tegra_safety_lic_logical_channel {
	uint16_t local_irq;
	uint8_t channel;
	bool is_vfiq;
};

/* Mapping of <irqs.h> numbers to LIC channels */
struct tegra_safety_lic_interrupt {
	uint8_t logical_channel;
	uint8_t bit;
	uint8_t slice;
};

struct tegra_safety_lic_id {
	struct tegra_safety_lic_logical_channel logical_channels[TEGRA_SAFETY_LIC_MAX_LOGICAL_CHANNELS];
	const struct tegra_safety_lic_interrupt *lic_map;
	uint16_t lic_irq_base;
	uint16_t lic_map_size;
};

struct tegra_safety_lic_regs {
	uint32_t channel;
	struct {
		uint32_t iep_class;
		volatile uint32_t ier;
	} reg[TEGRA_SAFETY_LIC_MAX_SLICES];
};

struct tegra_safety_lic_irq_context {
	struct tegra_safety_lic *lic;
	uint32_t irq_set;
	uint32_t channel;
	bool is_vfiq;
};

struct tegra_safety_lic_state {
	struct tegra_safety_lic_irq_context irq_context[TEGRA_SAFETY_LIC_MAX_LOGICAL_CHANNELS];
	struct tegra_safety_lic_regs shadow[TEGRA_SAFETY_LIC_MAX_CHANNELS];
	uint8_t shadow_bank[TEGRA_SAFETY_LIC_MAX_LOGICAL_CHANNELS];
	uint32_t num_banks;
	volatile uint32_t corrected_count;
	volatile uint32_t uncorrected_count;
	volatile uint32_t swerror_count;
};

struct tegra_safety_lic {
	const struct tegra_safety_lic_id *id;
	struct tegra_safety_lic_state state;
};

/* For compatibility */
#define TEGRA_LIC_BITSLICE(irq) \
	{ \
	.bit = (NV_ADDRESS_MAP_ ## irq ## _INTR_ID) % 32, \
	.slice = (NV_ADDRESS_MAP_ ## irq ## _INTR_ID) / 32, \
	}

#define TEGRA_LIC_INTERRUPT(ch, irq) \
	{ \
	.logical_channel = (ch), \
	.bit = (NV_ADDRESS_MAP_ ## irq ## _INTR_ID) % 32, \
	.slice = (NV_ADDRESS_MAP_ ## irq ## _INTR_ID) / 32, \
	}

#endif	/* INCLUDED_TEGRA_SAFETY_LIC_PRIV_H */
