/*
 * Copyright (c) 2014-2017 NVIDIA CORPORATION.  All rights reserved.
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

#include <araps_vic.h>

#include <reg-access.h>
#include <macros.h>
#include <arm-vic.h>

void arm_vic_enable(uint32_t vic_base, uint32_t irq)
{
	writel(BIT(irq), vic_base + APS_VIC_VICINTENABLE_0);
}

void arm_vic_disable(uint32_t vic_base, uint32_t irq)
{
	writel(BIT(irq), vic_base + APS_VIC_VICINTENCLEAR_0);
}

void arm_vic_disable_all(uint32_t vic_base)
{
	writel(~0U, vic_base + APS_VIC_VICINTENCLEAR_0);
}

void arm_vic_gen_software_int(uint32_t vic_base, uint32_t irq)
{
	writel(BIT(irq), vic_base + APS_VIC_VICSOFTINT_0);
}

void arm_vic_clear_software_int(uint32_t vic_base, uint32_t irq)
{
	writel(BIT(irq), vic_base + APS_VIC_VICSOFTINTCLEAR_0);
}

void arm_vic_set_isr_vect_addr(uint32_t vic_base, uint32_t irq,
				void (*isr_vect_addr)(void))
{
	uint32_t reg = APS_VIC_VICVECTADDR0_0 +
		(irq * (APS_VIC_VICVECTADDR1_0 - APS_VIC_VICVECTADDR0_0));

	writel((uint32_t)isr_vect_addr, vic_base + reg);
}

uint32_t arm_vic_read_irq_status(uint32_t vic_base)
{
	return readl(vic_base + APS_VIC_VICIRQSTATUS_0);
}

uint32_t arm_vic_read_fiq_status(uint32_t vic_base)
{
	return readl(vic_base + APS_VIC_VICFIQSTATUS_0);
}

uint32_t arm_vic_read_raw_int_status(uint32_t vic_base)
{
	return readl(vic_base + APS_VIC_VICRAWINTR_0);
}

void arm_vic_write_intselect(uint32_t vic_base, uint32_t intselect)
{
	writel(intselect, vic_base + APS_VIC_VICINTSELECT_0);
}

uint32_t arm_vic_read_intselect(uint32_t vic_base)
{
	return readl(vic_base + APS_VIC_VICINTSELECT_0);
}

void arm_vic_write_intenable(uint32_t vic_base, uint32_t intenable)
{
	writel(intenable, vic_base + APS_VIC_VICINTENABLE_0);
}

uint32_t arm_vic_read_intenable(uint32_t vic_base)
{
	return readl(vic_base + APS_VIC_VICINTENABLE_0);
}

void arm_vic_save_state(uint32_t vic_base, struct arm_vic_context *ctx)
{
	int i;

	for (i = 0; i < ARM_VIC_IRQ_COUNT; i++) {
		ctx->vect_addr[i] = readl(vic_base + APS_VIC_VICVECTADDR0_0 +
			i * (APS_VIC_VICVECTADDR1_0 - APS_VIC_VICVECTADDR0_0));
	}
	ctx->intenable = readl(vic_base + APS_VIC_VICINTENABLE_0);
	ctx->intselect = readl(vic_base + APS_VIC_VICINTSELECT_0);
}

void arm_vic_restore_state(uint32_t vic_base, struct arm_vic_context *ctx)
{
	int i;

	for (i = 0; i < ARM_VIC_IRQ_COUNT; i++) {
		writel(ctx->vect_addr[i], vic_base + APS_VIC_VICVECTADDR0_0 +
			i * (APS_VIC_VICVECTADDR1_0 - APS_VIC_VICVECTADDR0_0));
	}
	writel(~(ctx->intenable), vic_base + APS_VIC_VICINTENCLEAR_0);
	writel(ctx->intenable, vic_base + APS_VIC_VICINTENABLE_0);
	writel(ctx->intselect, vic_base + APS_VIC_VICINTSELECT_0);
}

#include "irqapi-arm.c"
