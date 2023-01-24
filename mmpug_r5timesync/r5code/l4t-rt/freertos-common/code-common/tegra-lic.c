/*
 * Copyright (c) 2016-2017, NVIDIA CORPORATION.  All rights reserved.
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

#include <tegra-lic.h>
#include <tegra-lic-priv.h>

#include <inttypes.h>
#include <stdint.h>
#include <string.h>

#include <address_map_new.h>
#include <arintr_ctlr.h>
#include <err-hook.h>
#include <irqs.h>
#include <macros.h>
#include <reg-access.h>

#include <printf-isr.h>

#ifdef DEBUG_TEGRA_LIC
#define dbgprintf(fmt, ...) printf_isr(fmt, ## __VA_ARGS__)
#else
#define dbgprintf(fmt, ...) ((void)0)
#endif

#define COMPAT_NUM_SLICES U32_C(9)
#define MAX_NUM_SLICES U32_C(11)

#define LIC_CHANNEL_OFFSET \
	(INTR_CTLR_CHANNEL1_SLICE0_VIRQ_0 - INTR_CTLR_CHANNEL0_SLICE0_VIRQ_0)
#define LIC_SLICE_OFFSET \
	(INTR_CTLR_CHANNEL0_SLICE1_VIRQ_0 - INTR_CTLR_CHANNEL0_SLICE0_VIRQ_0)

static inline uint32_t lic_slice_reg(const struct tegra_lic_id *id,
				uint32_t slice, uint32_t offset)
{
	return NV_ADDRESS_MAP_LIC_BASE +
		(id->base_channel * LIC_CHANNEL_OFFSET) +
		(slice * LIC_SLICE_OFFSET) +
		offset;
}

#define LIC_SLICE_REG(id, slice, reg) \
	lic_slice_reg((id), (slice), INTR_CTLR_CHANNEL0_SLICE0_ ## reg ## _0)
#define LIC_IER_SET(id, slice) LIC_SLICE_REG(id, slice, IER_SET)
#define LIC_IER_CLR(id, slice) LIC_SLICE_REG(id, slice, IER_CLR)
#define LIC_IER(id, slice) LIC_SLICE_REG(id, slice, IER)
#define LIC_IEP_CLASS(id, slice) LIC_SLICE_REG(id, slice, IEP_CLASS)
#define LIC_VIRQ(id, slice) LIC_SLICE_REG(id, slice, VIRQ)
#define LIC_VFIQ(id, slice) LIC_SLICE_REG(id, slice, VFIQ)

static inline uint32_t lic_num_slices(const struct tegra_lic_id *id)
{
	if (id->num_slices > 0U) {
		return id->num_slices;
	} else {
		return COMPAT_NUM_SLICES;
	}
}

void tegra_lic_enable(const struct tegra_lic_id *id, uint32_t irq)
{
	uint32_t lic_irq = irq - id->lic_irq_base;

	if (lic_irq >= id->lic_map_size) {
		error_hookf("LIC: trying to enable unmapped IRQ %"PRId32, irq);
		return;
	}

	dbgprintf("%s(%"PRIu32"): called ch=%"PRIu8"/%s intr_id=%"PRIu16"\r\n",
		__func__, irq, id->base_channel, id->is_vfiq ? "VFIQ" : "VIRQ",
		id->lic_map[lic_irq].slice * U16_C(32) + id->lic_map[lic_irq].bit);

	irq_enable(id->local_irq);

	dbgprintf("%s(%"PRIu32"): IER_SET[%"PRIu8"] to %"PRIu32"\r\n",
		__func__, irq, id->lic_map[lic_irq].slice, BIT(id->lic_map[lic_irq].bit));

	writel(BIT(id->lic_map[lic_irq].bit),
		LIC_IER_SET(id, id->lic_map[lic_irq].slice));
}

void tegra_lic_disable(const struct tegra_lic_id *id, uint32_t irq)
{
	uint32_t lic_irq = irq - id->lic_irq_base;

	if (lic_irq >= id->lic_map_size) {
		error_hookf("LIC: trying to disable unmapped IRQ %"PRId32, irq);
		return;
	}

	writel(BIT(id->lic_map[lic_irq].bit),
		LIC_IER_CLR(id, id->lic_map[lic_irq].slice));
}

static inline void tegra_lic_disable_strays(const struct tegra_lic_id *id,
					uint32_t virq[MAX_NUM_SLICES],
					uint32_t num_slices)
{
	bool stray = false;

	for (uint32_t i = 0; i < num_slices; i++) {
		if (virq[i] != 0) {
			writel(virq[i], LIC_IER_CLR(id, i));
			stray = true;
		}
	}

	if (!stray)
		return;

	for (uint32_t i = 0; i < num_slices; i++) {
		if (virq[i] == 0) {
			continue;
		}

		for (uint32_t j = 0; j < 32U; j++) {
			if ((BIT(j) & virq[i]) != 0U) {
				warning_hookf("stray LIC interrupt %"PRIu32
					" (ch=%"PRIu8"/%s), disabled",
					(i * 32U) + j, id->base_channel,
					id->is_vfiq ? "VFIQ" : "VIRQ");
			}
		}
	}
}

static void tegra_lic_handler(void *data)
{
	const struct tegra_lic_id *id = data;
	uint32_t reg = id->is_vfiq ? LIC_VFIQ(id, 0) : LIC_VIRQ(id, 0);
	uint32_t virq[MAX_NUM_SLICES];
	uint32_t num_slices = lic_num_slices(id);

	dbgprintf("%s(): called ch=%"PRIu8"/%s\r\n",
		__func__, id->base_channel, id->is_vfiq ? "VFIQ" : "VIRQ");

	for (uint32_t i = 0; i < num_slices; i++) {
		virq[i] = readl(reg + i * LIC_SLICE_OFFSET);
		if (virq[i] != 0) {
			dbgprintf("%s(ch=%"PRIu8"): %s[%"PRIu32"] has 0x%08"PRIx32"\r\n",
				__func__, id->base_channel,
				id->is_vfiq ? "VFIQ" : "VIRQ", i, virq[i]);
		}
	}

	for (uint32_t i = 0; i < id->lic_map_size; i++) {
		if (virq[id->lic_map[i].slice] & BIT(id->lic_map[i].bit)) {
			virq[id->lic_map[i].slice] &= ~BIT(id->lic_map[i].bit);
			irq_handler(id->lic_irq_base + i);
		}
	}

#ifdef TEGRA_LIC_DISABLE_STRAYS
	tegra_lic_disable_strays(id, virq, num_slices);
#endif
}

void tegra_lic_init(const struct tegra_lic_id *id)
{
	for (uint32_t i = 0; i < id->lic_map_size; i++) {
		uint8_t slice = id->lic_map[i].slice;
		uint32_t iep = readl(LIC_IEP_CLASS(id, slice));

		if (id->is_vfiq) {
			iep |= BIT(id->lic_map[i].bit);
		} else {
			iep &= ~BIT(id->lic_map[i].bit);
		}

		writel(iep, LIC_IEP_CLASS(id, slice));

		writel(BIT(id->lic_map[i].bit), LIC_IER_CLR(id, slice));
	}

	irq_set_handler(id->local_irq, tegra_lic_handler, (void *)id);
}
