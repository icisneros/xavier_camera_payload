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

/*
 *
 * This file fills in the hooks needed by libc. Libc functions like
 * malloc and printf.
 *
 */

#include <string.h>
#include <sys/stat.h>

#include <FreeRTOS.h>
#include <task.h>

#include <debug.h>

extern char _heap_start;
extern char _heap_end;
static char *_heap_watermark;

void *_sbrk(int incr);
void __malloc_lock(struct _reent *ptr);
void __malloc_unlock(struct _reent *ptr);
int _close(int file);
int _fstat(int file, struct stat *st);
int _isatty(int file);
int _lseek(int file, int ptr, int dir);
int _read(int file, char * ptr, int len);
int _write(int file, char *ptr, int len);

int _total_incr = 0;
void * __attribute__((weak)) _sbrk(int incr)
{
	char *brkval;

	if (!_heap_watermark)
		_heap_watermark = &_heap_start;

	if (((&_heap_end) - _heap_watermark) < incr)
		return (void *)-1;

	brkval = _heap_watermark;
	_heap_watermark += incr;

	memset(brkval, 0, incr);
	_total_incr += incr;

	return brkval;
}

void __attribute__((weak)) __malloc_lock(struct _reent *ptr)
{
	vTaskSuspendAll();
}

void __attribute__((weak)) __malloc_unlock(struct _reent *ptr)
{
	xTaskResumeAll();
}

int __attribute__((weak)) _close(int file)
{
	return -1;
}

int __attribute__((weak)) _fstat(int file, struct stat *st)
{
	st->st_mode = S_IFCHR;
	return 0;
}

int __attribute__((weak)) _isatty(int file)
{
	return 1;
}

int __attribute__((weak)) _lseek(int file, int ptr, int dir)
{
	return -1;
}

int __attribute__((weak)) _read(int file, char * ptr, int len)
{
	return -1;
}

int __attribute__((weak)) _write(int file, char *ptr, int len)
{
	if ((file != 1 && file != 2) || len <= 0)
		return -1;

	dbg_putd(ptr, len);

	return len;
}
