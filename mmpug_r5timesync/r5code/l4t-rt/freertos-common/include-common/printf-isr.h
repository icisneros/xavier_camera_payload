/*
 * Copyright (c) 2015, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _PRINTF_ISR_
#define _PRINTF_ISR_

#include <stdarg.h>

/*
 * This header defines printf_isr(), which is a printf implementation that
 * can be called from any context. It doesn't take locks or allocate memory.
 * Therefore, it is safe to call from interrupt handlers, unlike printf().
 *
 * Limitations:
 * - Output is limited to PRINTF_ISR_BUFSIZE characters. If necessary, this
 * may be increased, but you may have to increase the IRQ stack size too.
 * - You may not use the following specificers in the format string:
 * %a, %A, %S, %ls, or any other that uses multibyte character strings.
 * - There is no thread safety with respect to other printf() calls. Therefore,
 * it is possible that output from a printf() and a printf_isr() might be
 * mixed together.
 */

#define PRINTF_ISR_BUFSIZE 160

int printf_isr(const char *fmt, ...)
	__attribute__ ((format (__printf__, 1, 2)));

int vprintf_isr(const char *fmt, va_list ap)
	__attribute__ ((format (__printf__, 1, 0)));

#endif
