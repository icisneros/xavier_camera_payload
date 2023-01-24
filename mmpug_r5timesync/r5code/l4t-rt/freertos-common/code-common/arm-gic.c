/*
 * Copyright (c) 2014-2017, NVIDIA CORPORATION.  All rights reserved.
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

#include <arm-gic.h>
#include <macros.h>
#include <reg-access.h>

/* GIC distributor registers */
/* Distributor control register */
#define GICD_CTLR(_base_)			((_base_) + 0x000)
#define GICD_CTLR_ENABLE_GROUP1			BIT(1)
#define GICD_CTLR_ENABLE_GROUP0			BIT(0)
/* Type register */
#define GICD_TYPER(_base_)			((_base_) + 0x004)
#define GICD_TYPER_ITLINES_MASK			0x1f
#define GICD_TYPER_ITLINES_SHIFT		0
/* Group register (bit per IRQ) */
#define GICD_IGROUPR(_base_, _irq_)		((_base_) + 0x080 + (4 * ((_irq_) / 32)))
#define GICD_IGROUPR_SHIFT(_irq_)		((_irq_) % 32)
/* Set/Clear enable register (bit per IRQ) */
#define GICD_ISENABLER(_base_, _irq_)		((_base_) + 0x100 + (4 * ((_irq_) / 32)))
#define GICD_ICENABLER(_base_, _irq_)		((_base_) + 0x180 + (4 * ((_irq_) / 32)))
#define GICD_ISENABLER_SHIFT(_irq_)		((_irq_) % 32)
/* Set/Clear pending register (bit per IRQ) */
#define GICD_ISPEND(_base_, _irq_)		((_base_) + 0x200 + (4 * ((_irq_) / 32)))
#define GICD_ICPEND(_base_, _irq_)		((_base_) + 0x280 + (4 * ((_irq_) / 32)))
#define GICD_ISPEND_SHIFT(_irq_)		((_irq_) % 32)
/* Priority (byte per IRQ) */
#define GICD_IPRIORITYR(_base_, _irq_)		((_base_) + 0x400 + (_irq_))
/* Processor targets (byte bitmask per IRQ) */
#define GICD_ITARGETSR(_base_, _irq_)		((_base_) + 0x800 + (_irq_))
/* Configuration (2 bits per IRQ) */
#define GICD_ICFGR(_base_, _irq_)		((_base_) + 0xc00 + (4 * ((_irq_) / 16)))
#define GICD_ICFGR_SHIFT(_irq_)			((_irq_) % 16)

/* GIC CPU interface registers */
/* CPU interface control register */
#define GICC_CTLR(_base_)			((_base_) + 0x00)
#define GICC_CTLR_CBPR				BIT(4)
#define GICC_CTLR_FIQ_EN			BIT(3)
#define GICC_CTLR_ACK_CTL			BIT(2)
#define GICC_CTLR_ENABLE_GROUP1			BIT(1)
#define GICC_CTLR_ENABLE_GROUP0			BIT(0)
/* Interrupt priority mask register */
#define GICC_PMR(_base_)			((_base_) + 0x04)
/* Binary point register */
#define GICC_BPR(_base_)			((_base_) + 0x08)
/* Interrupt acknowledge register */
#define GICC_IAR(_base_)			((_base_) + 0x0c)
/* End of interrupt register */
#define GICC_EOIR(_base_)			((_base_) + 0x10)

static void arm_gic_init_dist(uint32_t dist_base, int cpunum)
{
	uint32_t val;
	uint32_t it_line_no;
	uint32_t irq;
	uint32_t mask;

	val = readl(GICD_TYPER(dist_base));
	it_line_no = ((val >> GICD_TYPER_ITLINES_SHIFT) &
		GICD_TYPER_ITLINES_MASK) + 1;

	for (irq = 0; irq < it_line_no * 32; irq += 32) {
		/* Enable all interrupts */
		writel(0xFFFFFFFF, GICD_ISENABLER(dist_base, irq));

		/* Disable all interrupts */
		writel(0xFFFFFFFF, GICD_ICENABLER(dist_base, irq));

		/* Make all interrupts NON-secure */
		writel(0xFFFFFFFF, GICD_IGROUPR(dist_base, irq));
	}

	/* Route all the interrupts except PPI's to this cpu */
	mask = BIT(cpunum);
	mask |= mask << 8 | mask << 16 | mask << 24;
	for (irq = 32; irq < it_line_no * 32; irq += 4)
		writel(mask, GICD_ITARGETSR(dist_base, irq));

	val = GICD_CTLR_ENABLE_GROUP1 | GICD_CTLR_ENABLE_GROUP0;
	writel(val, GICD_CTLR(dist_base));
}

void arm_gic_init_cpu(uint32_t cpu_base)
{
	uint32_t val;

	/* set interface priorty to max */
	writel(0xFF, GICC_PMR(cpu_base));

	/* disable interrupt premption */
	writel(0x07, GICC_BPR(cpu_base));

	val = GICC_CTLR_CBPR | GICC_CTLR_FIQ_EN | GICC_CTLR_ACK_CTL |
		GICC_CTLR_ENABLE_GROUP1 | GICC_CTLR_ENABLE_GROUP0;
	writel(val, GICC_CTLR(cpu_base));
}

void arm_gic_init(uint32_t dist_base, uint32_t cpu_base, int cpunum)
{
	/* Disable distributor, CPU interface */
	writel(0, GICD_CTLR(dist_base));
	writel(0, GICC_CTLR(cpu_base));

	arm_gic_init_dist(dist_base, cpunum);
	arm_gic_init_cpu(cpu_base);
}

void arm_gic_route_irq(uint32_t dist_base, uint32_t irq, bool edge, uint8_t cpu_mask, uint8_t priority)
{
	uint32_t val;

        /* disable irq */
	writel(BIT(GICD_ISENABLER_SHIFT(irq)), GICD_ICENABLER(dist_base, irq));

	/* set priority */
	writeb(priority, GICD_IPRIORITYR(dist_base, irq));

	/* set type of interrupt (level/edge) */
	val = readl(GICD_ICFGR(dist_base, irq));
	if (edge)
		val |= 0x2U << GICD_ICFGR_SHIFT(irq);
	else
		val &= ~(0x2U << GICD_ICFGR_SHIFT(irq));
	writel(val, GICD_ICFGR(dist_base, irq));

	/* set target CPU */
	writeb(cpu_mask, GICD_ITARGETSR(dist_base, irq));

	/* Make SPI interrupt to be configured as Group1 */
	if (irq > 31) {
		val = readl(GICD_IGROUPR(dist_base, irq));
		val |= (1U << GICD_IGROUPR_SHIFT(irq));
		writel(val, GICD_IGROUPR(dist_base, irq));
	}
}

void arm_gic_set_up_irq(uint32_t dist_base, uint32_t irq, bool edge, uint8_t cpu_mask, uint8_t priority)
{
	arm_gic_route_irq(dist_base, irq, edge, cpu_mask, priority);
	/* enable irq */
	writel(BIT(GICD_ISENABLER_SHIFT(irq)), GICD_ISENABLER(dist_base, irq));
}

void arm_gic_set_priority(uint32_t dist_base, uint32_t irq, uint8_t priority)
{
	writeb(priority, GICD_IPRIORITYR(dist_base, irq));
}

void arm_gic_enable(uint32_t dist_base, uint32_t irq)
{
	writel(BIT(GICD_ISENABLER_SHIFT(irq)), GICD_ISENABLER(dist_base, irq));
}

void arm_gic_disable(uint32_t dist_base, uint32_t irq)
{
	writel(BIT(GICD_ISENABLER_SHIFT(irq)), GICD_ICENABLER(dist_base, irq));
}

void arm_gic_set_pending(uint32_t dist_base, uint32_t irq)
{
	writel(BIT(GICD_ISPEND_SHIFT(irq)), GICD_ISPEND(dist_base, irq));
}

uint32_t arm_gic_iar_to_sgi_source_cpu(uint32_t iar)
{
	return (iar >> 10) & 0x7;
}

uint32_t arm_gic_iar_to_int_num(uint32_t iar)
{
	return iar & 0x3ff;
}

uint32_t arm_gic_read_iar(uint32_t cpu_base)
{
	return readl(GICC_IAR(cpu_base));
}

void arm_gic_write_eoi(uint32_t cpu_base, uint32_t iar)
{
	writel(iar, GICC_EOIR(cpu_base));
}

#include "irqapi-arm.c"
