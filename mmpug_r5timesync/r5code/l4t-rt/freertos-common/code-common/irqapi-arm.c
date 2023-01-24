/*
 * Copyright (c) 2015-2016 NVIDIA CORPORATION.  All rights reserved.
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

#include <irqs.h>

#define ARM_MODE_FIQ 0x11
#define ARM_MODE_IRQ 0x12
#define ARM_MODE_MASK 0x1F

static inline uint32_t arm_get_mode(void)
{
	uint32_t cpsr;

	__asm__ __volatile__ (
		"mrs %0, cpsr\n\t"
		: "=r" (cpsr)
	);

	return cpsr & ARM_MODE_MASK;
}

bool in_interrupt(void)
{
	uint32_t mode = arm_get_mode();
	return (mode == ARM_MODE_IRQ) || (mode == ARM_MODE_FIQ);
}

/*
 *** XXX: TEMPORARY ***
 *
 * This weak function is necessary until all FreeRTOS users include either
 * irqapi-cortex-r5.c or irqapi-cortex-a9.c
 *
 * Right now, udelay() is the only user of this function. Returning false here
 * will keep the same behavior as before.
 */
bool __attribute__((weak)) in_critical(void)
{
	return false;
}

/* Weak dummy implementation for FreeRTOS users with their own IRQ handling. */
__attribute__((weak))
void irq_set_handler(unsigned irq, void (*routine)(void *), void *opaque)
{
	(void) irq; (void) routine; (void) opaque;
}
