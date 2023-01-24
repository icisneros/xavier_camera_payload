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

#ifndef _ARM_VIC_H_
#define _ARM_VIC_H_

#define ARM_VIC_IRQ_COUNT 32

struct arm_vic_context {
	uint32_t vect_addr[ARM_VIC_IRQ_COUNT];
	uint32_t intenable;
	uint32_t intselect;
};

typedef void (*arm_vic_handler)(void);

void arm_vic_enable(uint32_t vic_base, uint32_t irq);
void arm_vic_disable(uint32_t vic_base, uint32_t irq);
void arm_vic_disable_all(uint32_t vic_base);
void arm_vic_gen_software_int(uint32_t vic_base, uint32_t irq);
void arm_vic_clear_software_int(uint32_t vic_base, uint32_t irq);
void arm_vic_set_isr_vect_addr(uint32_t vic_base, uint32_t irq, arm_vic_handler isr_vect_addr);
uint32_t arm_vic_read_irq_status(uint32_t vic_base);
uint32_t arm_vic_read_fiq_status(uint32_t vic_base);
uint32_t arm_vic_read_raw_int_status(uint32_t vic_base);
void arm_vic_write_intselect(uint32_t vic_base, uint32_t intselect);
uint32_t arm_vic_read_intselect(uint32_t vic_base);
void arm_vic_write_intenable(uint32_t vic_base, uint32_t intenable);
uint32_t arm_vic_read_intenable(uint32_t vic_base);
void arm_vic_save_state(uint32_t vic_base, struct arm_vic_context *ctx);
void arm_vic_restore_state(uint32_t vic_base, struct arm_vic_context *ctx);

#endif
