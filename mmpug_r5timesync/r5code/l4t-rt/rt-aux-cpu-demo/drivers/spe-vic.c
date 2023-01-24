/* Copyright (c) 2015-2016, NVIDIA CORPORATION. All rights reserved.
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
#include <address_map_new.h>

#include <arm-vic.h>
#include <macros.h>
#include <reg-access.h>

#include <irqs.h>
#include <spe-vic.h>

#define NUM_VIC_IRQS	32

extern uint32_t vic_base_addr[];

void spe_vic0_irq_handler(unsigned int irq)
{
	irq_handler(irq);

	writel(0, vic_base_addr[0] + APS_VIC_VICADDRESS_0);
}

void spe_vic1_irq_handler(unsigned int irq)
{
	irq_handler(irq + 32);

	writel(0, vic_base_addr[1] + APS_VIC_VICADDRESS_0);
	writel(0, vic_base_addr[0] + APS_VIC_VICADDRESS_0);
}

extern void spe_vic0_vectors(void);
extern void spe_vic0_vectors_end(void);
extern void spe_vic1_vectors(void);
extern void spe_vic1_vectors_end(void);

void spe_vic_init(void)
{
	uint32_t vic0_vector_size;
	uint32_t vic1_vector_size;
	int i;
	void (*vic0_vector)(void) = spe_vic0_vectors;
	void (*vic1_vector)(void) = spe_vic1_vectors;

	vic0_vector_size = ((uint32_t)spe_vic0_vectors_end - (uint32_t)spe_vic0_vectors) / NUM_VIC_IRQS;
	vic1_vector_size = ((uint32_t)spe_vic1_vectors_end - (uint32_t)spe_vic1_vectors) / NUM_VIC_IRQS;

	for (i = 0; i < 32; i++) {
		arm_vic_set_isr_vect_addr(vic_base_addr[0], i, vic0_vector);
		arm_vic_set_isr_vect_addr(vic_base_addr[1], i, vic1_vector);
		vic0_vector = (void (*)(void))((uint32_t) vic0_vector + vic0_vector_size);
		vic1_vector = (void (*)(void))((uint32_t) vic1_vector + vic1_vector_size);
	}
}
