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

#ifndef INCLUDED_ARM_GIC_H
#define INCLUDED_ARM_GIC_H

#include <stdbool.h>
#include <stdint.h>

#define ARM_GIC_IRQ_BASE_SGI  0
#define ARM_GIC_IRQ_BASE_PPI 16
#define ARM_GIC_IRQ_BASE_SPI 32

/*
 * Perform all initialization steps required for other GIC-related
 * functionality to operate.
 *
 * This routes all SPIs to the provided cpunum. PPIs and SGIs are not routable
 * in HW. All IRQs are masked (disabled). Both IRQ and FIQ are enabled in both
 * the distributor and CPU interface.
 *
 * Parameters:
 * dist_base:	The base address of the GIC distributor registers.
 * cpu_base:	The base address of the GIC CPU interface registers.
 * cpunum:	The CPU number to route all SPIs to.
 */
void arm_gic_init(uint32_t dist_base, uint32_t cpu_base, int cpunum);

/*
 * Perform all initialization steps required for other GIC-related
 * functionality to operate. Do not route any interrupts.
 */
void arm_gic_init_cpu(uint32_t cpu_base);

/*
 * Configure an IRQ's level-/edge-type, target CPU, priority and disable the IRQ.
 *
 * Parameters:
 * dist_base:	The base address of the GIC distributor registers.
 * irq:		The controller-relative IRQ number.
 * edge:	True if the IRQ is edge sensitive, false if level sensitive.
 * cpumask:	The bitmask of CPUs to route the IRQ to.
 * priority:	The priority for the IRQ.
 */
void arm_gic_route_irq(uint32_t dist_base, uint32_t irq, bool edge,
		uint8_t cpu_mask, uint8_t priority);

/*
 * Configure an IRQ's level-/edge-type, target CPU, priority and enable the IRQ
 *
 * Parameters:
 * dist_base:	The base address of the GIC distributor registers.
 * irq:		The controller-relative IRQ number.
 * edge:	True if the IRQ is edge sensitive, false if level sensitive.
 * cpumask:	The bitmask of CPUs to route the IRQ to.
 * priority:	The priority for the IRQ.
 */
void arm_gic_set_up_irq(uint32_t dist_base, uint32_t irq, bool edge,
	uint8_t cpu_mask, uint8_t priority);

/*
 * Configure an IRQ's priority.
 *
 * Parameters:
 * dist_base:	The base address of the GIC distributor registers.
 * irq:		The controller-relative IRQ number.
 * priority:	The priority for the IRQ.
 */
void arm_gic_set_priority(uint32_t dist_base, uint32_t irq, uint8_t priority);

/*
 * Enable an IRQ.
 *
 * Parameters:
 * dist_base:	The base address of the GIC distributor registers.
 * irq:		The controller-relative IRQ number.
 */
void arm_gic_enable(uint32_t dist_base, uint32_t irq);

/*
 * Disable an IRQ.
 *
 * Parameters:
 * dist_base:	The base address of the GIC distributor registers.
 * irq:		The controller-relative IRQ number.
 */
void arm_gic_disable(uint32_t dist_base, uint32_t irq);

/*
 * Force an IRQ to be pending.
 *
 * Parameters:
 * dist_base:	The base address of the GIC distributor registers.
 * irq:		The controller-relative IRQ number.
 */
void arm_gic_set_pending(uint32_t dist_base, uint32_t irq);

/*
 * Extract the source CPU ID from a value read from the GIC CPU interface's
 * IAR register. This assumes that the IAR value represents an SGI (software
 * generated interrupt).
 *
 * Parameters:
 * iar:		A value read from the IAR register.
 */
uint32_t arm_gic_iar_to_sgi_source_cpu(uint32_t iar);

/*
 * Extract the controller-relative IRQ number from a value read from the GIC
 * CPU interface's IAR register.
 *
 * Parameters:
 * iar:		A value read from the IAR register.
 */
uint32_t arm_gic_iar_to_int_num(uint32_t iar);

/*
 * Read the GIC CPU interface's IAR register.
 *
 * Parameters:
 * cpu_base:	The base address of the GIC CPU interface registers.
 *
 * Returns:
 * ARM_GIC_IAR_SPURIOS: Spurious interrupt (deasserted before reading IAR)
 * >= ARM_GIC_IAR_SECURE: Secure interrupts (unused if secure interrupts are unimplemented)
 * Otherwise: interrupt number
 */
uint32_t arm_gic_read_iar(uint32_t cpu_base);

#define ARM_GIC_IAR_SECURE (1020U)
#define ARM_GIC_IAR_SPURIOS (1023U)

/*
 * Write the GIC CPU interface's EOI register to signify that handling of an
 * IRQ is complete.
 *
 * Parameters:
 * cpu_base:	The base address of the GIC CPU interface registers.
 * iar:		The value previously read from the IAR register.
 */
void arm_gic_write_eoi(uint32_t cpu_base, uint32_t iar);

#endif /* INCLUDED_ARM_GIC_H */
