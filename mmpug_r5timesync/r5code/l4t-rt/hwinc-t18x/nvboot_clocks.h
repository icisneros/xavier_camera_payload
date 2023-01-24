/*
 * Copyright (c) 2007 - 2009 NVIDIA Corporation.  All rights reserved.
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

/**
 * Defines useful constants for working with clocks.
 */


#ifndef INCLUDED_NVBOOT_CLOCKS_H
#define INCLUDED_NVBOOT_CLOCKS_H

#if defined(__cplusplus)
extern "C"
{
#endif

/*
 * Set of valid count ranges per frequency.
 * Measuring 13 gives exactly 406 e.g.
 * The chosen range parameter is:
 * - more than the expected combined frequency deviation
 * - less than half the  relative distance between 12 and 13
 * - expressed as a ratio against a power of two to avoid floating point
 * - so that intermediate overflow is not possible
 *
 * The chosen factor is 1/64 or slightly less than 1.6% = 2^-6
 * Rounding is performed in such way as to guarantee at least the range
 * that is down for min and up for max
 * the range macros receive the frequency in kHz as argument
 * division by 32 kHz then becomes a shift by 5 to the right
 *
 * The macros are defined for a frequency of 32768 Hz (not 32000 Hz).
 * They use 2^-5 ranges, or about 3.2% and dispense with the rounding.
 * Also need to use the full value in Hz in the macro
 */

#define NVBOOT_CLOCKS_MIN_RANGE(F) (( F - (F>>5) - (1<<15) + 1 ) >> 15)
#define NVBOOT_CLOCKS_MAX_RANGE(F) (( F + (F>>5) + (1<<15) - 1 ) >> 15)

// For an easier ECO (keeping same number of instructions), we need a
// special case for 12 min range
#define NVBOOT_CLOCKS_MIN_CNT_12 (NVBOOT_CLOCKS_MIN_RANGE(12000000) -1)
#define NVBOOT_CLOCKS_MAX_CNT_12 NVBOOT_CLOCKS_MAX_RANGE(12000000)

#define NVBOOT_CLOCKS_MIN_CNT_13 NVBOOT_CLOCKS_MIN_RANGE(13000000)
#define NVBOOT_CLOCKS_MAX_CNT_13 NVBOOT_CLOCKS_MAX_RANGE(13000000)

#define NVBOOT_CLOCKS_MIN_CNT_16_8 NVBOOT_CLOCKS_MIN_RANGE(16800000)
#define NVBOOT_CLOCKS_MAX_CNT_16_8 NVBOOT_CLOCKS_MAX_RANGE(16800000)

#define NVBOOT_CLOCKS_MIN_CNT_19_2 NVBOOT_CLOCKS_MIN_RANGE(19200000)
#define NVBOOT_CLOCKS_MAX_CNT_19_2 NVBOOT_CLOCKS_MAX_RANGE(19200000)

#define NVBOOT_CLOCKS_MIN_CNT_26 NVBOOT_CLOCKS_MIN_RANGE(26000000)
#define NVBOOT_CLOCKS_MAX_CNT_26 NVBOOT_CLOCKS_MAX_RANGE(26000000)

#define NVBOOT_CLOCKS_MIN_CNT_38_4 NVBOOT_CLOCKS_MIN_RANGE(38400000)
#define NVBOOT_CLOCKS_MAX_CNT_38_4 NVBOOT_CLOCKS_MAX_RANGE(38400000)

#define NVBOOT_CLOCKS_MIN_CNT_48 NVBOOT_CLOCKS_MIN_RANGE(48000000)
#define NVBOOT_CLOCKS_MAX_CNT_48 NVBOOT_CLOCKS_MAX_RANGE(48000000)

// The stabilization delay in usec
#define NVBOOT_CLOCKS_PLL_STABILIZATION_DELAY (300)
// The stabilization delay after the lock bit indicates the PLL is lock
#define NVBOOT_CLOCKS_PLL_STABILIZATION_DELAY_AFTER_LOCK (10)

#if defined(__cplusplus)
}
#endif

#endif /* #ifndef INCLUDED_NVBOOT_CLOCKS_H */
