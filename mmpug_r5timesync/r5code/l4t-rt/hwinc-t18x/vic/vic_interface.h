//
// Copyright (c) 2018 NVIDIA Corporation.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// Neither the name of the NVIDIA Corporation nor the names of its contributors
// may be used to endorse or promote products derived from this software
// without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

//
// Driver-ucode interface data structures and defines

#ifndef __VICDRVAPPCOMMON_H__
#define __VICDRVAPPCOMMON_H__

typedef unsigned long long      UInt64;
typedef   signed long long       Int64;
typedef unsigned long           UInt32;
typedef   signed long            Int32;
typedef unsigned short          UInt16;
typedef   signed short           Int16;
typedef unsigned char            UInt8;
typedef   signed char             Int8;
typedef unsigned long             UInt;
typedef   signed long              Int;
typedef          char             Char;
typedef unsigned char             Byte;

typedef unsigned char             Bool;
typedef          void             Void;
typedef          float           Float;
typedef          double         Double;
typedef          char          *String;

typedef volatile unsigned long        VUInt;
typedef volatile long                 VInt;

#define False           0
#define True            1

#define _U32_(x)               ((UInt32)(x))
#define _U_(x)                 _U32_(x)
#define stdROUNDDOWN(a,b)     ((_U_(a)/_U_(b))*(_U_(b)))
#define stdROUNDUP(a,b)       stdROUNDDOWN(_U_(a)+_U_(b)-1,_U_(b))

#if defined (VIC_4_1)
#define DEF(x)                  NVB1B6##x
#define CLASS_ID                0xB1B6
#define MAX_SLOT_COUNT          16
#else
#define DEF(x)                  NVB0B6##x
#define CLASS_ID                0xB0B6
#define MAX_SLOT_COUNT          8
#endif

#define METHOD(x)           DEF(x)
#define ERROR(x)            (VIC_ERROR##x)
/*****************************************************************************/
// DMA Ids (vld, col, hist, and dbf ctxdma id's are predefined by hw (see hwref_mv.h)
#define VIC_CTXID_CONFIG            1
#define VIC_CTXID_PALETTE           2
#define VIC_CTXID_HIST              3
#define VIC_CTXID_FB                4
#define VIC_CTXID_UCODE             5

#define ENABLE_INTERRUPT_HANDLING

// structs for parameter passing, these are all multiple of 128 bits large
// the sizes of the member are chosen so that each bit vector is aligned within that member, i.e. no space is wasted

#define __int64 long long int
#include "vic_structs.h"
#include "vic_enums.h"

// error codes
#define VIC_ERROR_INVALID_CTXID_SURFACE(i,j)                    (((CLASS_ID << 16) | 0x0000) | (i<<4) | j)
#define VIC_ERROR_INVALID_CTXID_FCE_UCODE                       ((CLASS_ID << 16) | 0x0050)
#define VIC_ERROR_INVALID_CTXID_CONFIG                          ((CLASS_ID << 16) | 0x0051)
#define VIC_ERROR_INVALID_CTXID_PALETTE                         ((CLASS_ID << 16) | 0x0052)
#define VIC_ERROR_INVALID_CTXID_HIST                            ((CLASS_ID << 16) | 0x0053)
#define VIC_ERROR_INVALID_CTXID_OUT                             ((CLASS_ID << 16) | 0x0054)
#define VIC_ERROR_CONFIG_STRUCT_SIZE_MISMATCH                   ((CLASS_ID << 16) | 0x0060)
#define VIC_ERROR_INVALID_INPUT_FORMAT_SLOT(i)                  (((CLASS_ID << 16) | 0x0070) | i)
#define VIC_ERROR_INVALID_OUTPUT_FORMAT                         ((CLASS_ID << 16) | 0x0080)
#define VIC_ERROR_APPTIMER_EXPIRED                              ((CLASS_ID << 16) | 0x0090)
// interruts
#define VIC_ERROR_FCE_PANIC_ERRINTR                             ((CLASS_ID << 16) | 0x00A0)
#define VIC_ERROR_FCE_DIVISION_BY_ZERO_ERRINTR                  ((CLASS_ID << 16) | 0x00A5)
#define VIC_ERROR_SC_BANK_CONFLICT_ERRINT                       ((CLASS_ID << 16) | 0x00A1)
#define VIC_ERROR_SC_RESET_ERRINT                               ((CLASS_ID << 16) | 0x00A2)
#define VIC_ERROR_YS_SKIPD_ERRINT                               ((CLASS_ID << 16) | 0x00A3)
#define VIC_ERROR_XS_SKIPD_ERRINT                               ((CLASS_ID << 16) | 0x00A4)


#define CONFIG_STRUCT_SIZE (sizeof(ConfigStruct))

#define HISTOGRAM_DATA_OFFSET       (0x100 >> 8)

#define DIFF_INDEX           0
#define WEAVE_INDEX          1
#define PWEAVE_INDEX         2
#define CADENCE_INDEX        3
#define NEW_MAX_CTRL_COUNT   2
#define MAX_CTRL_COUNT       4


#define FMT_A8R8G8B8        T_A8R8G8B8
#define FMT_X8R8G8B8        T_X8R8G8B8
#define FMT_A2R10G10B10     T_A2R10G10B10
#define FMT_B8G8R8A8        T_B8G8R8A8
#define FMT_A8L8            T_A8L8
#define FMT_NV12            T_Y8___V8U8_N420
#define FMT_10BIT_NV12      T_Y10___V10U10_N420
#define FMT_12BIT_NV12      T_Y12___V12U12_N420
#define FMT_YV12            T_Y8___U8___V8_N420
#define FMT_10BIT_YV12      T_Y10___U10___V10_N420
#define FMT_12BIT_YV12      T_Y12___U12___V12_N420
#define FMT_UYVY            T_U8_Y8__V8_Y8
#define FMT_YUY2            T_Y8_U8__Y8_V8
#define PIXEL_FORMAT_MAX    78

#endif  // #ifndef _MP4DRVAPPCOMMON_H_

