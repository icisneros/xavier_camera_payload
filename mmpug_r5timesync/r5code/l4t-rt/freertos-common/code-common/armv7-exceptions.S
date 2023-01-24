@
@ Copyright (c) 2015-2016, NVIDIA CORPORATION.  All rights reserved.
@
@ Redistribution and use in source and binary forms, with or without
@ modification, are permitted provided that the following conditions
@ are met:
@  * Redistributions of source code must retain the above copyright
@    notice, this list of conditions and the following disclaimer.
@  * Redistributions in binary form must reproduce the above copyright
@    notice, this list of conditions and the following disclaimer in the
@    documentation and/or other materials provided with the distribution.
@  * Neither the name of NVIDIA CORPORATION nor the names of its
@    contributors may be used to endorse or promote products derived
@    from this software without specific prior written permission.
@
@ THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
@ EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
@ IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
@ PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
@ CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
@ EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
@ PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
@ PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
@ OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
@ (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
@ OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
@

#include <armv7-exception.h>

#define MODE_MASK 0x1f
#define MODE_FIQ 0x11
#define MODE_IRQ 0x12
#define MODE_SVC 0x13
#define MODE_ABT 0x17
#define MODE_UND 0x1b
#define MODE_SYS 0x1f

#define THUMB_BIT 0x20

.extern armv7_exception

.section .text

.global _Data_Abort_Handler
_Data_Abort_Handler:
	sub lr, lr, #8
	stmfd sp!, {r0-r12, lr}
	mov r0, #DATA_ABORT
	b common_handler

.global _Prefetch_Abort_Handler
_Prefetch_Abort_Handler:
	sub lr, lr, #4
	stmfd sp!, {r0-r12, lr}
	mov r0, #PREFETCH_ABORT
	b common_handler

.global _Undefined_Instruction_Handler
_Undefined_Instruction_Handler:
	stmfd sp!, {r0-r12, lr}
	mov r0, #UNDEFINED_INSTRUCTION
	@ Update lr:
	sub lr, lr, #4
	mrs r4, spsr
	tst r4, #THUMB_BIT
	beq 1f
	@ Thumb mode, fix lr offset:
	add lr, lr, #2
1:
	str lr, [sp, #52]
	b common_handler

@ r0: exception type
@ r1: exception frame/saved registers
@ r2: temporary cpsr to change modes
@ r3: saved cpsr
@ r4: spsr
common_handler:
	mrs r4, spsr
	stmfd sp!, {r4}
	mrs r3, cpsr
	bic r2, r3, #MODE_MASK
	mov r1, sp

	bic r4, #~MODE_MASK
	teq r4, #MODE_FIQ
	beq mode_fiq
	teq r4, #MODE_IRQ
	beq mode_irq
	teq r4, #MODE_SVC
	beq mode_svc
	teq r4, #MODE_ABT
	beq mode_abt
	teq r4, #MODE_UND
	beq mode_und

	@ If M field isn't valid, just pretend it's SYS
	orr r2, r2, #MODE_SYS
	b common_end

mode_fiq:
	orr r2, r2, #MODE_FIQ
	@ FIQ mode also has banked r8-r12. We should fix the saved registers.
	msr cpsr_c, r2
	add r5, r1, #36
	stmea r5, {r8-r12}
	b common_end

mode_irq:
	orr r2, r2, #MODE_IRQ
	b common_end

mode_svc:
	orr r2, r2, #MODE_SVC
	b common_end

mode_abt:
	orr r2, r2, #MODE_ABT
	b common_end

mode_und:
	orr r2, r2, #MODE_UND

common_end:
	msr cpsr_c, r2
	stmfd r1!, {sp, lr}
	msr cpsr_c, r3
	@ Need to subtract 4 for alignment
	sub sp, r1, #4
	blx armv7_exception
	b .
