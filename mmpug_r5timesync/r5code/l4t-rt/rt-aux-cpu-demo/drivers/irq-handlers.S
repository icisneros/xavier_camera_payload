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

.macro vic0_irq_handlers, num
.if \num
	vic0_irq_handlers "(\num-1)"

	push	{r0-r3, ip, lr}
	bl	spe_vic0_dispatch
.endif
.endm

.macro vic1_irq_handlers, num
.if \num
	vic1_irq_handlers "(\num-1)"

	push	{r0-r3, ip, lr}
	bl	spe_vic1_dispatch
.endif
.endm

	.section .text
	.global spe_vic0_vectors
	.global spe_vic0_vectors_end
	.global spe_vic1_vectors
	.global spe_vic1_vectors_end
spe_vic0_dispatch:
	adr	r0, (spe_vic0_vectors + 8)
	sub	r0, lr, r0
	lsr	r0, r0, #3
	bl	spe_vic0_irq_handler
	b	irq_end
spe_vic0_vectors:
	vic0_irq_handlers	32
spe_vic0_vectors_end:
spe_vic1_dispatch:
	adr	r0, (spe_vic1_vectors + 8)
	sub	r0, lr, r0
	lsr	r0, r0, #3
	bl	spe_vic1_irq_handler
	b	irq_end
spe_vic1_vectors:
	vic1_irq_handlers	32
spe_vic1_vectors_end:
