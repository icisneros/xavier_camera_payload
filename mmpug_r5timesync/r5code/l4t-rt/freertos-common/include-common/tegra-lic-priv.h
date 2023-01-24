/*
 * Copyright (c) 2016-2018 NVIDIA CORPORATION.  All rights reserved.
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

#ifndef INCLUDED_LIC_TEGRA_PRIV_H
#define INCLUDED_LIC_TEGRA_PRIV_H

#include <stdint.h>
#include <tegra-lic.h>

/* Mapping of <irqs.h> numbers to LIC IRQs */
struct tegra_lic_bitslice {
	uint8_t channel;
	uint8_t bit;
	uint8_t slice;
	uint8_t is_fiq;
};

struct tegra_lic_id {
	uint8_t base_channel;
	uint8_t num_channels;
	uint8_t num_slices;
	uint8_t is_vfiq;
	uint16_t local_irq;
	uint16_t lic_irq_base;
	uint16_t lic_map_size;
	const struct tegra_lic_bitslice *lic_map;
};

#define TEGRA_LIC_BITSLICE(irq) \
	{ \
	.bit = (NV_ADDRESS_MAP_ ## irq ## _INTR_ID) % 32, \
	.slice = (NV_ADDRESS_MAP_ ## irq ## _INTR_ID) / 32, \
	}

#endif	/* INCLUDED_LIC_TEGRA_PRIV_H */
