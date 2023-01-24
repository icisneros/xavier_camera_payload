/*
 * Copyright (c) 2008 - 2009 NVIDIA Corporation.  All rights reserved.
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
 * Defines the parameters and data structure for MuxOneNAND and
 * FlexMuxOneNAND devices.
 */

#ifndef INCLUDED_NVBOOT_MUX_ONE_NAND_PARAM_H
#define INCLUDED_NVBOOT_MUX_ONE_NAND_PARAM_H

#include "nvcommon.h"

#if defined(__cplusplus)
extern "C"
{
#endif

/**
 * Defines the clock sources that can be selected for MuxOneNAND and
 * FlexMuxOneNAND devices.
 */
/*
 * The order of the enum MUST match the HW definition for the first
 * four values.  Note that this is a violation of the SW convention on enums.
 */
typedef enum
{
    /// Specifies PLLP as the clock source
    NvBootMuxOneNandClockSource_PllPOut0 = 0,

    /// Specifies PLLC as the clock source
    NvBootMuxOneNandClockSource_PllCOut0,

    /// Specifies PLLM as the clock source
    NvBootMuxOneNandClockSource_PllMOut0,

    /// Specifies ClockM as the clock source
    NvBootMuxOneNandClockSource_ClockM,

    NvBootMuxOneNandClockSource_Num,
    NvBootMuxOneNandClockSource_Force32 = 0x7FFFFFF
} NvBootMuxOneNandClockSource;

/// Defines the params that can be configured for MuxOneNAND and
/// FlexMuxOneNAND devices.
typedef struct NvBootMuxOneNandParamsRec
{
    /// Specifies the clock source for the MuxOneNand/SNOR controller
    NvBootMuxOneNandClockSource ClockSource;

    /// Specifies the clock divider for the chosen clock source
    NvU32 ClockDivider;

} NvBootMuxOneNandParams;

#if defined(__cplusplus)
}
#endif

#endif /* #ifndef INCLUDED_NVBOOT_MUX_ONE_NAND_PARAM_H */
