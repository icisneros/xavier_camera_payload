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

#ifndef INCLUDED_ERR_HOOK_H
#define INCLUDED_ERR_HOOK_H

#include <stdbool.h>

/*
 * This header defines APIs that code can use to report errors or warnings
 * that are discovered at run-time.
 *
 * This can be especially useful when an error is detected by code that has no
 * way to report the error. For example, if an ISR attempts to send a message
 * to a task so that it can handle the interrupt, but sending the message
 * fails.
 *
 * It is expected that a variety of back-ends will implement these functions.
 * One might simply printf the message to a log/UART for a developer to see.
 * Another might reboot the CPU/SoC if a fatal error is detected, in order to
 * attempt recovery.
 *
 * Irrespective of the backend, a developer might set a breakpoint in these
 * functions in order to see whenever any kind of problem occurred.
 */

/*
 * Don't call these; they're just protoypes for the user accessible functions
 * below to use.
 */
void bug_hook_imp(const char *file, int line, const char *function,
	const char *msg)
	__attribute__ ((noreturn));
void bug_hookf_imp(const char *file, int line, const char *function,
	const char *msg, ...)
	__attribute__ ((format (__printf__, 4, 5)))
	__attribute__ ((noreturn));
void error_hook_imp(const char *file, int line, const char *function,
	const char *msg);
void error_hookf_imp(const char *file, int line, const char *function,
	const char *msg, ...)
	__attribute__ ((format (__printf__, 4, 5)));
void error_hookf_once_imp(bool *once,
	const char *file, int line, const char *function, const char *msg, ...)
	__attribute__ ((format (__printf__, 5, 6)));
void warning_hook_imp(const char *file, int line, const char *function,
	const char *msg);
void warning_hookf_imp(const char *file, int line, const char *function,
	const char *msg, ...)
	__attribute__ ((format (__printf__, 4, 5)));
void warning_hookf_once_imp(bool *once,
	const char *file, int line, const char *function, const char *msg, ...)
	__attribute__ ((format (__printf__, 5, 6)));

/*
 * The following are macros so that we can later decide to conditionally
 * remove the parameters for production code (e.g. #indef DEBUG), if we wish.
 */

#ifndef FILENAME
#define FILENAME __FILE__
#endif

/*
 * Halt CPU until reset
 */
void halt_on_bug(void)
	__attribute__ ((noreturn));

/*
 * Report that an fatal error occurred, along with a brief description of the
 * problem.
 */
#define bug_hook(msg) \
	bug_hook_imp(FILENAME, __LINE__, __func__, msg)

#define bug_hookf(...) \
	bug_hookf_imp(FILENAME, __LINE__, __func__, __VA_ARGS__)

/*
 * Report that an error occurred, along with a brief description of the
 * problem.
 */
#define error_hook(msg) \
	error_hook_imp(FILENAME, __LINE__, __func__, msg)

#define error_hookf(...) \
	error_hookf_imp(FILENAME, __LINE__, __func__, __VA_ARGS__)

#define error_hookf_once(...) do { \
		static bool once_ ## __LINE__; \
		if (!once_ ##__LINE__) { \
			error_hookf_once_imp(&once_ ## __LINE__, \
					FILENAME, __LINE__, __func__, \
					__VA_ARGS__); \
		} \
	} while(false)

/*
 * Report that a non-fatal problem occurred, along with a brief description of
 * the problem.
 */
#define warning_hook(msg) \
	warning_hook_imp(FILENAME, __LINE__, __func__, msg)

#define warning_hookf(...) \
	warning_hookf_imp(FILENAME, __LINE__, __func__, __VA_ARGS__)

#define warning_hookf_once(...) do { \
		static bool once_ ## __LINE__;	\
		if (!once_ ##__LINE__) { \
			warning_hookf_once_imp(&once_ ## __LINE__, \
					FILENAME, __LINE__, __func__, \
					__VA_ARGS__); \
		}; \
	} while(false)

#endif	/* INCLUDED_ERR_HOOK_H */
