/*
 * Copyright (c) 2015-2018, NVIDIA CORPORATION.  All rights reserved.
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

#include <stdio.h>
#include <stdarg.h>

#include <err-hook.h>
#include <irqs.h>
#include <printf-isr.h>

#include <FreeRTOS.h>
#include <task.h>

static void err_msg(const char *err_type, const char *file, int line,
		const char *function, const char *msg)
{
	if (in_critical()) {
		printf_isr("%s %s:%d [%s] \"", err_type, file, line, function);
		printf_isr("%s\"\r\n", msg);
	} else {
		printf("%s %s:%d [%s] \"", err_type, file, line, function);
		printf("%s\"\r\n", msg);
	}
}

__attribute__ ((format (__printf__, 5, 0)))
static void err_printf(const char *err_type, const char *file, int line,
		const char *function, const char *msg, va_list ap)
{
	if (in_critical()) {
		printf_isr("%s %s:%d [%s] \"", err_type, file, line, function);
	        vprintf_isr(msg, ap);
		printf_isr("\"\r\n");
	} else {
		printf("%s %s:%d [%s] \"", err_type, file, line, function);
	        vprintf(msg, ap);
		printf("\"\r\n");
	}
}

__attribute__ ((weak))
void halt_on_bug(void)
{
	/* Prevent interrupts from firing */
	vPortEnterCritical();

	for (;;) {
	}
}

void bug_hook_imp(const char *file, int line, const char *function,
	const char *msg)
{
	err_msg("BUG:", file, line, function, msg);
	halt_on_bug();
}

void bug_hookf_imp(const char *file, int line, const char *function,
	const char *msg, ...)
{
	va_list args;
	va_start(args, msg);
	err_printf("BUG:", file, line, function, msg, args);
	va_end(args);
	halt_on_bug();
}

void error_hookf_imp(const char *file, int line, const char *function,
	const char *msg, ...)
{
	va_list args;
	va_start(args, msg);
	err_printf("ERROR:", file, line, function, msg, args);
	va_end(args);
}

static inline bool has_been_done(bool *once)
{
	if (in_interrupt()) {
		if (*once) {
			return true;
		}

		*once = true;

		return false;
	} else {
		taskENTER_CRITICAL();

		if (*once) {
			taskEXIT_CRITICAL();
			return true;
		}

		*once = true;

		taskEXIT_CRITICAL();

		return false;
	}
}

void error_hookf_once_imp(bool *once,
		const char *file, int line, const char *function,
		const char *msg, ...)
{
	if (!has_been_done(once)) {
		va_list args;
		va_start(args, msg);
		err_printf("ERROR:", file, line, function, msg, args);
		va_end(args);
	}
}

void warning_hookf_imp(const char *file, int line, const char *function,
	const char *msg, ...)
{
	va_list args;
	va_start(args, msg);
	err_printf("WARNING:", file, line, function, msg, args);
	va_end(args);
}

void warning_hookf_once_imp(bool *once,
		const char *file, int line, const char *function,
		const char *msg, ...)
{
	if (!has_been_done(once)) {
		va_list args;
		va_start(args, msg);
		err_printf("WARNING:", file, line, function, msg, args);
		va_end(args);
	}
}

void error_hook_imp(const char *file, int line, const char *function,
	const char *msg)
{
	err_msg("ERROR:", file, line, function, msg);
}

void warning_hook_imp(const char *file, int line, const char *function,
	const char *msg)
{
	err_msg("WARNING:", file, line, function, msg);
}
