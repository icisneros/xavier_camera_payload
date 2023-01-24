/*
 * Copyright (c) 2015-2017 NVIDIA CORPORATION.  All rights reserved.
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

#include <stdint.h>

#include <hsp-tegra.h>
#include <hsp-tegra-hw.h>
#include <hsp-tegra-priv.h>
#include <irqs.h>
#include <macros.h>
#include <reg-access.h>

#include <address_map_new.h>
#include <arhsp_int.h>
#include <arhsp_dbell.h>
#include <arhsp_shrd_mbox.h>
#include <arhsp_shrd_sem.h>

#define HSP_COMMON_SIZE (NV_ADDRESS_MAP_TOP0_HSP_COMMON_SIZE)

#define HSP_INT_IE_OFFSET (HSP_INT_IE_1 - HSP_INT_IE_0)
#define SI_INDEX	0 /* use first IRQ for now */

#define HSP_SHRD_MBOX_OFFSET \
        (HSP_SHRD_MBOX1_FIRST_REG - HSP_SHRD_MBOX0_FIRST_REG)

#define HSP_SHRD_MBOX_TAG_FIELD HSP_SHRD_MBOX_MBOX_0_SHRD_MBOX_0_TAG_FIELD
#define HSP_SHRD_MBOX_DATA_FIELD HSP_SHRD_MBOX_MBOX_0_SHRD_MBOX_0_DATA_FIELD

#define HSP_DBELL_OFFSET (HSP_DBELL_1_TRIGGER_0 - HSP_DBELL_0_TRIGGER_0)

#define HSP_INT_DIMENSIONING_FIELD(regval, field) \
        (((regval) >> HSP_INT_DIMENSIONING_0_ ## field ## _SHIFT) & \
                (HSP_INT_DIMENSIONING_0_ ## field ## _FIELD >> \
                        HSP_INT_DIMENSIONING_0_ ## field ## _SHIFT))

#define HSP_SHRD_SEM_STA	HSP_SHRD_SEM_0_SHRD_SMP_STA_0
#define HSP_SHRD_SEM_SET	HSP_SHRD_SEM_0_SHRD_SMP_STA_SET_0
#define HSP_SHRD_SEM_CLR	HSP_SHRD_SEM_0_SHRD_SMP_STA_CLR_0
#define HSP_SHRD_SEM_OFFSET	(HSP_SHRD1_FIRST_REG - HSP_SHRD0_FIRST_REG)

static inline struct tegra_hsp_ctx *hsp_ctx(const struct tegra_hsp_id *id)
{
	/* struct tegra_hsp_id assumed to be first field in tegra_hsp_ctx */
	return (struct tegra_hsp_ctx *)id;
}

static void tegra_hsp_si_writel(struct tegra_hsp_ctx *ctx, uint32_t index,
				uint32_t value)
{
	const struct tegra_hsp_id *id = &ctx->id;

	writel(value, id->base_addr + HSP_INT_IE +
                (HSP_INT_IE_OFFSET * index));
}

static uint32_t tegra_hsp_si_readl(struct tegra_hsp_ctx *ctx, uint32_t index)
{
	const struct tegra_hsp_id *id = &ctx->id;

	return readl(id->base_addr + HSP_INT_IE +
                     (HSP_INT_IE_OFFSET * index));
}

static void tegra_hsp_sm_writel(struct tegra_hsp_ctx *ctx, uint32_t index,
				uint32_t value)
{
	const struct tegra_hsp_id *id = &ctx->id;

	writel(value, id->base_addr + HSP_COMMON_SIZE +
               (HSP_SHRD_MBOX_OFFSET * index));
}

static uint32_t tegra_hsp_sm_readl(struct tegra_hsp_ctx *ctx, uint32_t index)
{
	const struct tegra_hsp_id *id = &ctx->id;

	return readl(id->base_addr + HSP_COMMON_SIZE +
                     (HSP_SHRD_MBOX_OFFSET * index));
}

static void tegra_hsp_db_writel(struct tegra_hsp_ctx *ctx, uint32_t host,
				uint32_t value, uint32_t offset)
{
	writel(value, ctx->db_base + (HSP_DBELL_OFFSET * host) + offset);
}

static uint32_t tegra_hsp_db_readl(struct tegra_hsp_ctx *ctx, uint32_t host,
					uint32_t offset)
{
	return readl(ctx->db_base + (HSP_DBELL_OFFSET * host) + offset);
}

static void tegra_hsp_db_self_writel(struct tegra_hsp_ctx *ctx,
					uint32_t value, uint32_t offset)
{
	const struct tegra_hsp_id *id = &ctx->id;

	tegra_hsp_db_writel(ctx, id->host, value, offset);
}

static uint32_t tegra_hsp_db_self_readl(struct tegra_hsp_ctx *ctx,
					uint32_t offset)
{
	const struct tegra_hsp_id *id = &ctx->id;

	return tegra_hsp_db_readl(ctx, id->host, offset);
}

void tegra_hsp_db_irq_handler(void *data)
{
	struct tegra_hsp_ctx *ctx = data;
	uint32_t db_pending;
	BaseType_t higher_prio_task_woken = pdFALSE;

	db_pending = tegra_hsp_db_self_readl(ctx, HSP_DBELL_0_PENDING_0);
	tegra_hsp_db_self_writel(ctx, db_pending, HSP_DBELL_0_PENDING_0);

	while (db_pending) {
		uint32_t source = __builtin_ffs(db_pending) - 1;
		db_pending &= ~BIT(source);
		ctx->db_callback(source, &higher_prio_task_woken);
	}

	portYIELD_FROM_ISR(higher_prio_task_woken);
}

enum {
	tegra_hsp_sm_none,
	tegra_hsp_sm_full,
	tegra_hsp_sm_empty,
};

static void tegra_hsp_irq_handler(void *data)
{
	struct tegra_hsp_ctx *ctx = data;

	for (unsigned i = 0; i < ctx->n_sm; i++) {
		void *cb_data = ctx->sm[i].opaque;
		uint32_t value;

		switch (ctx->cbtype[i]) {
		case tegra_hsp_sm_full:
			value = tegra_hsp_sm_readl(ctx, i);
			if ((value & HSP_SHRD_MBOX_TAG_FIELD) != 0) {
				ctx->sm[i].callback(cb_data, value);
			}
			break;
		case tegra_hsp_sm_empty:
			value = tegra_hsp_sm_readl(ctx, i);
			if ((value & HSP_SHRD_MBOX_TAG_FIELD) == 0) {
				ctx->sm[i].callback(cb_data, value);
			}
			break;
		case tegra_hsp_sm_none:
		default:
			break;
		}
	}
}

void tegra_hsp_init(const struct tegra_hsp_id *id)
{
	struct tegra_hsp_ctx *ctx = hsp_ctx(id);
	uint32_t val;

	val = readl(id->base_addr + HSP_INT_DIMENSIONING_0);

	ctx->n_sm = HSP_INT_DIMENSIONING_FIELD(val, nSM);
	ctx->n_ss = HSP_INT_DIMENSIONING_FIELD(val, nSS);
	ctx->n_as = HSP_INT_DIMENSIONING_FIELD(val, nAS);
	ctx->n_db = HSP_INT_DIMENSIONING_FIELD(val, nDB);
	ctx->n_si = HSP_INT_DIMENSIONING_FIELD(val, nSI);
	ctx->db_base = id->base_addr +
		((1 + (ctx->n_sm / 2) + ctx->n_ss + ctx->n_as) << 16u);

	if (id->db_irq >= 0) {
		irq_set_handler(id->db_irq, tegra_hsp_db_irq_handler, ctx);
		irq_enable(id->db_irq);
	}

	if (id->sh_irqs[SI_INDEX] >= 0) {
		irq_set_handler(id->sh_irqs[SI_INDEX], tegra_hsp_irq_handler,
				ctx);
		irq_enable(id->sh_irqs[SI_INDEX]);
	}
}

int tegra_hsp_db_init(const struct tegra_hsp_id *id, uint32_t enabled_masters,
                tegra_hsp_callback callback)
{
	/* struct tegra_hsp_id assumed to be first field in tegra_hsp_ctx */
	struct tegra_hsp_ctx *ctx = hsp_ctx(id);

	if (!callback)
		return 1;

	ctx->db_callback = callback;

	tegra_hsp_init(id);

	/* Enable the interrupt from <enabled_master> */
	tegra_hsp_db_self_writel(ctx, enabled_masters, HSP_DBELL_0_ENABLE_0);

	return 0;
}

void tegra_hsp_sm_produce(const struct tegra_hsp_id *id, uint32_t sm,
				int32_t value)
{
	struct tegra_hsp_ctx *ctx = hsp_ctx(id);

	tegra_hsp_sm_writel(ctx, sm, HSP_SHRD_MBOX_TAG_FIELD | (uint32_t)value);
}

int32_t tegra_hsp_sm_consume(const struct tegra_hsp_id *id, uint32_t sm)
{
	int32_t v = tegra_hsp_sm_peek(id, sm);

	if (v >= 0)
		tegra_hsp_sm_vacate(id, sm);

	return v;
}

bool tegra_hsp_sm_is_empty(const struct tegra_hsp_id *id, uint32_t sm)
{
	struct tegra_hsp_ctx *ctx = hsp_ctx(id);
	uint32_t v = tegra_hsp_sm_readl(ctx, sm);

	return (v & HSP_SHRD_MBOX_TAG_FIELD) == 0;
}

int32_t tegra_hsp_sm_peek(const struct tegra_hsp_id *id, uint32_t sm)
{
	struct tegra_hsp_ctx *ctx = hsp_ctx(id);
	uint32_t v = tegra_hsp_sm_readl(ctx, sm);

	if ((v & HSP_SHRD_MBOX_TAG_FIELD) == 0)
		return -1; /* mailbox is empty */

	return v & HSP_SHRD_MBOX_DATA_FIELD;
}

void tegra_hsp_sm_vacate(const struct tegra_hsp_id *id, uint32_t sm)
{
	struct tegra_hsp_ctx *ctx = hsp_ctx(id);

	tegra_hsp_sm_writel(ctx, sm, 0);
}

static void tegra_hsp_sm_enable(const struct tegra_hsp_id *id,
				uint32_t sm, unsigned setbit, uint8_t cbtype,
				void (*cb)(void *, uint32_t), void *data)
{
	struct tegra_hsp_ctx *ctx = hsp_ctx(id);
	uint32_t ie;

	if (!in_interrupt())
		taskENTER_CRITICAL();

	ie = tegra_hsp_si_readl(ctx, SI_INDEX);

	ctx->sm[sm].callback = cb;
	ctx->sm[sm].opaque = data;
	ctx->cbtype[sm] = cbtype;

	tegra_hsp_si_writel(ctx, SI_INDEX, ie | BIT(setbit));

	if (!in_interrupt())
		taskEXIT_CRITICAL();
}

static void tegra_hsp_sm_disable(const struct tegra_hsp_id *id,
				uint32_t sm, unsigned clrbit)
{
	struct tegra_hsp_ctx *ctx = hsp_ctx(id);
	uint32_t ie;

	if (!in_interrupt())
		taskENTER_CRITICAL();

	ie = tegra_hsp_si_readl(ctx, SI_INDEX);
	tegra_hsp_si_writel(ctx, SI_INDEX, ie & ~BIT(clrbit));

	/* FIXME: sync interrupt handler here */
	ctx->sm[sm].callback = NULL;
	ctx->cbtype[sm] = tegra_hsp_sm_none;

	if (!in_interrupt())
		taskEXIT_CRITICAL();
}

void tegra_hsp_sm_full_enable(const struct tegra_hsp_id *id, uint32_t sm,
				void (*cb)(void *, uint32_t), void *data)
{
	unsigned iebit = HSP_INT_IE_0_mbox_full_enable_SHIFT + sm;
	tegra_hsp_sm_enable(id, sm, iebit, tegra_hsp_sm_full, cb, data);
}

void tegra_hsp_sm_full_disable(const struct tegra_hsp_id *id, uint32_t sm)
{
	unsigned iebit = HSP_INT_IE_0_mbox_full_enable_SHIFT + sm;
	tegra_hsp_sm_disable(id, sm, iebit);
}

void tegra_hsp_sm_empty_enable(const struct tegra_hsp_id *id, uint32_t sm,
				void (*cb)(void *, uint32_t), void *data)
{
	unsigned iebit = HSP_INT_IE_0_mbox_empty_enable_SHIFT + sm;
	tegra_hsp_sm_enable(id, sm, iebit, tegra_hsp_sm_empty, cb, data);
}

void tegra_hsp_sm_empty_disable(const struct tegra_hsp_id *id, uint32_t sm)
{
	unsigned iebit = HSP_INT_IE_0_mbox_empty_enable_SHIFT + sm;
	tegra_hsp_sm_disable(id, sm, iebit);
}

void tegra_hsp_db_ring(const struct tegra_hsp_id *id, uint32_t target)
{
	struct tegra_hsp_ctx *ctx = hsp_ctx(id);

	tegra_hsp_db_writel(ctx, target, 1, HSP_DBELL_0_TRIGGER_0);
}

void tegra_hsp_db_enable_master(const struct tegra_hsp_id *id,
				uint32_t enabled_masters)
{
	struct tegra_hsp_ctx *ctx = hsp_ctx(id);
	uint32_t enable;

	if (!in_interrupt())
		taskENTER_CRITICAL();

	enable = tegra_hsp_db_self_readl(ctx, HSP_DBELL_0_ENABLE_0);
	enable |= enabled_masters;
	tegra_hsp_db_self_writel(ctx, enable, HSP_DBELL_0_ENABLE_0);

	if (!in_interrupt())
		taskEXIT_CRITICAL();
}

void tegra_hsp_db_disable_master(const struct tegra_hsp_id *id,
					uint32_t disabled_masters)
{
	struct tegra_hsp_ctx *ctx = hsp_ctx(id);
	uint32_t enable;

	if (!in_interrupt())
		taskENTER_CRITICAL();

	enable = tegra_hsp_db_self_readl(ctx, HSP_DBELL_0_ENABLE_0);
	enable &= ~disabled_masters;
	tegra_hsp_db_self_writel(ctx, enable, HSP_DBELL_0_ENABLE_0);

	if (!in_interrupt())
		taskEXIT_CRITICAL();
}

int tegra_hsp_suspend(const struct tegra_hsp_id *id,
				struct tegra_hsp_suspend_ctx *sctx)
{
	struct tegra_hsp_ctx *ctx = hsp_ctx(id);

	if (id->sh_irqs[SI_INDEX] >= 0) {
		if (sctx)
			sctx->si_enable[SI_INDEX] = tegra_hsp_si_readl(ctx, SI_INDEX);
	}
	if (id->db_irq >= 0) {
		if (sctx)
			sctx->db_enable = tegra_hsp_db_self_readl(ctx, HSP_DBELL_0_ENABLE_0);
	}

	return 0;
}

int tegra_hsp_resume(const struct tegra_hsp_id *id,
			struct tegra_hsp_suspend_ctx *sctx)
{
	struct tegra_hsp_ctx *ctx = hsp_ctx(id);

	if (id->db_irq >= 0) {
		if (sctx)
			tegra_hsp_db_self_writel(ctx, sctx->db_enable, HSP_DBELL_0_ENABLE_0);
		irq_enable(id->db_irq);
	}
	if (id->sh_irqs[SI_INDEX] >= 0) {
		if (sctx)
			tegra_hsp_si_writel(ctx, SI_INDEX, sctx->si_enable[SI_INDEX]);
		irq_enable(id->sh_irqs[SI_INDEX]);
	}

	return 0;
}

static inline uint32_t tegra_hsp_ss_reg(const struct tegra_hsp_ctx *ctx,
					int index, uint32_t reg)
{
	const struct tegra_hsp_id *id = &ctx->id;
	uint32_t addr = id->base_addr + HSP_COMMON_SIZE +
					(HSP_SHRD_MBOX_OFFSET * ctx->n_sm) +
					(HSP_SHRD_SEM_OFFSET * index) +
					reg;
	return addr;
}

int tegra_hsp_ss_read(const struct tegra_hsp_id *id, uint32_t index, uint32_t *value)
{
	uint32_t addr;
	struct tegra_hsp_ctx *ctx = hsp_ctx(id);

	if (index >= ctx->n_ss) {
		return -1;
	}

	addr = tegra_hsp_ss_reg(ctx, index, HSP_SHRD_SEM_STA);
	*value = readl(addr);

	return 0;
}

int tegra_hsp_ss_set(const struct tegra_hsp_id *id, uint32_t index, uint32_t data)
{
	uint32_t addr;
	struct tegra_hsp_ctx *ctx = hsp_ctx(id);

	if (index >= ctx->n_ss) {
		return -1;
	}

	addr = tegra_hsp_ss_reg(ctx, index, HSP_SHRD_SEM_SET);
	writel(data, addr);

	return 0;
}

int tegra_hsp_ss_clear(const struct tegra_hsp_id * id, uint32_t index, uint32_t data)
{
	uint32_t addr;
	struct tegra_hsp_ctx *ctx = hsp_ctx(id);

	if (index >= ctx->n_ss) {
		return -1;
	}

	addr = tegra_hsp_ss_reg(ctx, index, HSP_SHRD_SEM_CLR);
	writel(data, addr);

	return 0;
}
