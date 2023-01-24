/*
 * Copyright (c) 2014-2018, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _MACROS_H_
#define _MACROS_H_

#include <inttypes.h>

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#define ARRAY_SSIZE(x) (int)(sizeof(x) / sizeof((x)[0]))

#define SIZE_K(KB) (1024U * (KB))
#define SIZE_M(MB) SIZE_K(1024U * (MB))

/* Turn on one bit in a 32-bit word */
#define BIT(_x_) ((uint32_t)(1U) << (_x_))
#define BIT32(_x_) ((uint32_t)(1U) << (_x_))

/* Turn on one bit in a 64-bit word */
#define BIT64(_x_) ((uint64_t)(1U) << (_x_))

#define DIV_ROUND_UP(x, y) (((x) + (y) - 1U) / (y))

#define min(a, b) (((a) < (b)) ? (a) : (b))

#define max(a, b) (((a) > (b)) ? (a) : (b))

#define CONTAINER_OF(_ptr, _type, _field) \
	(_type *)((char *)_ptr - offsetof(_type, _field))

#ifndef U64_C
#define U64_C(x) UINT64_C(x)
#endif

#ifndef U32_C
#define U32_C(x) UINT32_C(x)
#endif

#ifndef U16_C
#define U16_C(x) UINT16_C(x)
#endif

#ifndef U8_C
#define U8_C(x) UINT8_C(x)
#endif

#endif
