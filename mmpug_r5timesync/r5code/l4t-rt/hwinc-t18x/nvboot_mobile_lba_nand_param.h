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
 * Defines the parameters and data structure for mobileLBA NAND devices.
 */

#ifndef INCLUDED_NVBOOT_MOBILE_LBA_NAND_PARAM_H
#define INCLUDED_NVBOOT_MOBILE_LBA_NAND_PARAM_H

#if defined(__cplusplus)
extern "C"
{
#endif

/// Params that are used to config and initialize MobileLbaNand driver.
typedef struct NvBootMobileLbaNandParamsRec
{
    /**
     * Specifies the clock divider for the PLL_P 432MHz source.
     * If it is set to 18, then clock source to Nand controller is
     * 432 / 18 = 24MHz.
     */
    NvU8 ClockDivider;

    /// Specifies the value to be programmed to Nand Timing Register 1
    NvU32 NandTiming;

    /// Specifies the value to be programmed to Nand Timing Register 2
    NvU32 NandTiming2;

    /**
     * Specifies the number of sectors that can be used from Sda region. If the
     * read request falls beyond this sector count, the request will be
     * diverted to Mba region.
     */
    NvU32 SdaSectorCount;
} NvBootMobileLbaNandParams;

#if defined(__cplusplus)
}
#endif

#endif /* #ifndef INCLUDED_NVBOOT_MOBILE_LBA_NAND_PARAM_H */
