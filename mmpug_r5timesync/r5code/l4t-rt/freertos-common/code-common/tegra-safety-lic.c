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

#include <FreeRTOS.h>
#include <task.h>
#include <tegra-safety-lic.h>
#include <tegra-safety-lic-priv.h>

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

#ifdef DEBUG_TEGRA_SAFETY_LIC
#define dbgprintf(fmt, ...) printf_isr(fmt, ## __VA_ARGS__)
#else
#define dbgprintf(fmt, ...) ((void)0)
#endif

#define MAX_NUM_CHANNELS	TEGRA_SAFETY_LIC_MAX_CHANNELS
#define MAX_NUM_SLICES		TEGRA_SAFETY_LIC_MAX_SLICES
#define MAX_LOGICAL_CHANNELS	TEGRA_SAFETY_LIC_LOGICAL_CHANNELS

#define LIC_CHANNEL_OFFSET \
	(INTR_CTLR_CHANNEL1_SLICE0_VIRQ_0 - INTR_CTLR_CHANNEL0_SLICE0_VIRQ_0)
#define LIC_SLICE_OFFSET \
	(INTR_CTLR_CHANNEL0_SLICE1_VIRQ_0 - INTR_CTLR_CHANNEL0_SLICE0_VIRQ_0)

static inline uint32_t lic_reg(uint32_t offset)
{
	return NV_ADDRESS_MAP_LIC_BASE + offset;
}

static inline uint32_t lic_channel_slice_reg(uint8_t channel, uint8_t slice, uint32_t offset)
{
	return lic_reg(
		((uint32_t)channel * LIC_CHANNEL_OFFSET) +
		((uint32_t)slice * LIC_SLICE_OFFSET) +
		offset);
}

#define LIC_CHANNEL_SLICE_REG(channel, slice, reg)			\
	lic_channel_slice_reg((channel), (slice), INTR_CTLR_CHANNEL0_SLICE0_ ## reg ## _0)

#define LIC_IER(channel, slice) LIC_CHANNEL_SLICE_REG(channel, slice, IER)
#define LIC_IEP_CLASS(channel, slice) LIC_CHANNEL_SLICE_REG(channel, slice, IEP_CLASS)
#define LIC_VIRQ(channel, slice) LIC_CHANNEL_SLICE_REG(channel, slice, VIRQ)
#define LIC_VFIQ(channel, slice) LIC_CHANNEL_SLICE_REG(channel, slice, VFIQ)

static void safe_writel(struct tegra_safety_lic *lic, uint32_t value, uint32_t addr)
{
	uint32_t v;

	dbgprintf("%s(0x%"PRIx32", 0x%"PRIx32")\r\n", __func__, value, addr);

	writel(value, addr);
	v = readl(addr);

	if (v != value) {
		writel(value, addr);
		v = readl(addr);

		if (v != value) {
			lic->state.uncorrected_count++;
		} else {
			lic->state.corrected_count++;
		}

		dbgprintf("%s: corrected = %"PRIu32", uncorrected = %"PRIu32"\r\n",
			__func__, lic->state.corrected_count,
			lic->state.uncorrected_count);
	}
}

void tegra_safety_lic_enable(struct tegra_safety_lic *lic, uint32_t irq)
{
	const struct tegra_safety_lic_id *id = lic->id;
	struct tegra_safety_lic_state *state = &lic->state;
	uint32_t lic_irq = irq - id->lic_irq_base;

	if (lic_irq >= id->lic_map_size) {
		dbgprintf("LIC: trying to enable unmapped IRQ %"PRIu32"\r\n", irq);
		lic->state.swerror_count++;
		return;
	}

	uint32_t lch = id->lic_map[lic_irq].logical_channel;

	if (lch >= MAX_LOGICAL_CHANNELS) {
		dbgprintf("LIC: trying to enable unused IRQ %"PRIu32"\r\n", irq);
		lic->state.swerror_count++;
		return;
	}

	uint32_t channel = id->logical_channels[lch].channel;
	uint32_t slice = id->lic_map[lic_irq].slice;
	uint32_t bit = id->lic_map[lic_irq].bit;
	uint32_t mask = BIT(bit);
	struct tegra_safety_lic_regs *shadow = &state->shadow[state->shadow_bank[lch]];

	dbgprintf("%s(%"PRIu32"): channel=%"PRIu32"/%s intr_id=%"PRIu32"\r\n",
		__func__, irq, channel, id->logical_channels[lch].is_vfiq ? "VFIQ" : "VIRQ",
		slice * U32_C(32) + bit);

	dbgprintf("%s(%"PRIu32"): IER_SET[%"PRIu32"] := 0x%"PRIx32"\r\n",
		__func__, irq, slice, BIT(bit));

	if (!in_interrupt()) {
		taskENTER_CRITICAL();
	}

	shadow->reg[slice].ier |= mask;
	safe_writel(lic, shadow->reg[slice].ier, LIC_IER(channel, slice));

	if (!in_interrupt()) {
		taskEXIT_CRITICAL();
	}
}

void tegra_safety_lic_disable(struct tegra_safety_lic *lic, uint32_t irq)
{
	const struct tegra_safety_lic_id *id = lic->id;
	struct tegra_safety_lic_state *state = &lic->state;
	uint32_t lic_irq = irq - id->lic_irq_base;

	if (lic_irq >= id->lic_map_size) {
		dbgprintf("LIC: trying to disable unmapped IRQ %"PRIu32"\r\n", irq);
		lic->state.swerror_count++;
		return;
	}

	uint32_t lch = id->lic_map[lic_irq].logical_channel;

	if (lch >= MAX_LOGICAL_CHANNELS) {
		dbgprintf("LIC: trying to disable unused IRQ %"PRIu32"\r\n", irq);
		lic->state.swerror_count++;
		return;
	}

	uint32_t channel = id->logical_channels[lch].channel;
	uint32_t slice = id->lic_map[lic_irq].slice;
	uint32_t bit = id->lic_map[lic_irq].bit;
	uint32_t mask = BIT(bit);
	struct tegra_safety_lic_regs *shadow = &state->shadow[state->shadow_bank[lch]];

	dbgprintf("%s(%"PRIu32"): channel=%"PRIu32"/%s intr_id=%"PRIu32"\r\n",
		__func__, irq, channel, id->logical_channels[lch].is_vfiq ? "VFIQ" : "VIRQ",
		slice * U32_C(32) + bit);

	dbgprintf("%s(%"PRIu32"): IER_SET[%"PRIu32"] := 0x%"PRIx32"\r\n",
		__func__, irq, slice, BIT(bit));

	if (!in_interrupt()) {
		taskENTER_CRITICAL();
	}

	shadow->reg[slice].ier &= ~mask;
	safe_writel(lic, shadow->reg[slice].ier, LIC_IER(channel, slice));

	if (!in_interrupt()) {
		taskEXIT_CRITICAL();
	}
}

static void tegra_safety_lic_handler(void *data)
{
	struct tegra_safety_lic_irq_context *ctx = data;
	const struct tegra_safety_lic_id *id = ctx->lic->id;
	uint32_t channel = ctx->channel;
	uint32_t reg = ctx->is_vfiq ? LIC_VFIQ(channel, 0) : LIC_VIRQ(channel, 0);
	uint32_t virq[MAX_NUM_SLICES];
	uint32_t slice_map = U32_C(0);

	dbgprintf("%s(): called ch=%"PRIu32"/%s\r\n",
		__func__, channel, ctx->is_vfiq ? "VFIQ" : "VIRQ");

	uint32_t irqs_left = ctx->irq_set;
	while (irqs_left) {
		uint32_t lic_irq = (uint32_t)__builtin_ctz(irqs_left);
		uint32_t slice = id->lic_map[lic_irq].slice;
		uint32_t mask = BIT(id->lic_map[lic_irq].bit);
		uint32_t irq = id->lic_irq_base + lic_irq;

		if ((slice_map & BIT(slice)) == U32_C(0)) {
			virq[slice] = readl(reg + slice * LIC_SLICE_OFFSET);
			slice_map |= BIT(slice);

			dbgprintf("%s(ch=%"PRIu32"): %s[%"PRIu32"] 0x%08"PRIx32"\r\n",
				__func__, channel,
				ctx->is_vfiq ? "VFIQ" : "VIRQ", slice, virq[slice]);
		}

		if ((virq[slice] & mask) != U32_C(0)) {
			irq_handler(irq);
		}

		irqs_left &= ~BIT(lic_irq);
	}
}

void tegra_safety_lic_init(struct tegra_safety_lic *lic)
{
	const struct tegra_safety_lic_id *id = lic->id;
	struct tegra_safety_lic_state *state = &lic->state;
	uint8_t lch, channel, slice;

	(void)memset(state, 0, sizeof(*state));

	for (uint32_t irq = 0; irq < id->lic_map_size; irq++) {
		lch = id->lic_map[irq].logical_channel;
		slice = id->lic_map[irq].slice;

		if ((lch >= MAX_LOGICAL_CHANNELS) || (slice >= MAX_NUM_SLICES)) {
			/* Disabled */
			continue;
		}

		channel = id->logical_channels[lch].channel;

		uint8_t bank;
		for (bank = 0; bank < state->num_banks; bank++) {
			if (channel == state->shadow[bank].channel) {
				break;
			}
		}

		if (bank >= MAX_NUM_CHANNELS) {
			state->swerror_count++;
			return;
		}

		if (bank == state->num_banks) {
			state->shadow_bank[lch] = bank;
			state->shadow[bank].channel = channel;
			state->num_banks++;
		}

		if (id->logical_channels[lch].is_vfiq) {
			bank = state->shadow_bank[lch];
			state->shadow[bank].reg[slice].iep_class |= BIT(id->lic_map[irq].bit);
		}

		state->irq_context[lch].irq_set |= BIT(irq);
	}

	/* Program register file */
	tegra_safety_lic_restore_config(lic);

	for (lch = 0; lch < ARRAY_SIZE(id->logical_channels); lch++) {
		channel = id->logical_channels[lch].channel;
		bool is_vfiq = id->logical_channels[lch].is_vfiq;

		struct tegra_safety_lic_irq_context *ctx = &state->irq_context[lch];
		ctx->lic = lic;
		ctx->channel = channel;
		ctx->is_vfiq = is_vfiq;

		uint16_t local_irq = id->logical_channels[lch].local_irq;
		irq_set_handler(local_irq, tegra_safety_lic_handler, ctx);
		irq_enable(local_irq);
	}
}

void tegra_safety_lic_check_config(struct tegra_safety_lic *lic)
{
	struct tegra_safety_lic_state *state = &lic->state;
	struct tegra_safety_lic_regs *shadow = state->shadow;

	for (uint32_t bank = 0; bank < state->num_banks; bank++) {
		uint32_t channel = shadow[bank].channel;

		for (uint32_t slice = U32_C(0); slice < MAX_NUM_SLICES; slice++) {

			taskENTER_CRITICAL();

			uint32_t ier = readl(LIC_IER(channel, slice));

			if (ier != shadow[bank].reg[slice].ier) {
				state->uncorrected_count++;
			}

			taskEXIT_CRITICAL();

			uint32_t iep_class = readl(LIC_IEP_CLASS(channel, slice));

			if (iep_class != shadow[bank].reg[slice].iep_class) {
				state->uncorrected_count++;
			}
		}
	}

	dbgprintf("%s: corrected = %"PRIu32", uncorrected = %"PRIu32"\r\n",
		__func__, state->corrected_count, state->uncorrected_count);
}

void tegra_safety_lic_restore_config(struct tegra_safety_lic *lic)
{
	struct tegra_safety_lic_state *state = &lic->state;
	struct tegra_safety_lic_regs *shadow = state->shadow;

	for (uint32_t bank = 0; bank < state->num_banks; bank++) {
		uint32_t channel = shadow[bank].channel;

		for (uint32_t slice = U32_C(0); slice < MAX_NUM_SLICES; slice++) {

			taskENTER_CRITICAL();

			safe_writel(lic, U32_C(0), LIC_IER(channel, slice));

			safe_writel(lic, shadow[bank].reg[slice].iep_class,
				LIC_IEP_CLASS(channel, slice));

			safe_writel(lic, shadow[bank].reg[slice].ier,
				LIC_IER(channel, slice));

			taskEXIT_CRITICAL();
		}
	}
}

uint32_t tegra_safety_lic_get_corrected_count(const struct tegra_safety_lic *lic)
{
	return lic->state.corrected_count;
}

void tegra_safety_lic_clear_corrected_count(struct tegra_safety_lic *lic)
{
	lic->state.corrected_count = U32_C(0);
}

uint32_t tegra_safety_lic_get_uncorrected_count(const struct tegra_safety_lic *lic)
{
	return lic->state.uncorrected_count;
}

void tegra_safety_lic_clear_uncorrected_count(struct tegra_safety_lic *lic)
{
	lic->state.uncorrected_count = U32_C(0);
}

uint32_t tegra_safety_lic_get_swerror_count(const struct tegra_safety_lic *lic)
{
	return lic->state.swerror_count;
}

void tegra_safety_lic_clear_swerror_count(struct tegra_safety_lic *lic)
{
	lic->state.swerror_count = U32_C(0);
}
