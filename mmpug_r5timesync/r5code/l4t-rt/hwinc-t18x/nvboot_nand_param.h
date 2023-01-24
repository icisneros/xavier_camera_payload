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
 * Defines the parameters and data structure for NAND devices.
 */

#ifndef INCLUDED_NVBOOT_NAND_PARAM_H
#define INCLUDED_NVBOOT_NAND_PARAM_H

#if defined(__cplusplus)
extern "C"
{
#endif

/// Defines the params that can be configured for NAND devices.
typedef struct NvBootNandParamsRec
{
    /**
     * Specifies the clock divider for the PLL_P 432MHz source.
     * If it is set to 18, then clock source to Nand controller is
     * 432 / 18 = 24MHz.
     */

    NvU8 ClockDivider;

    /// Specifies the value to be programmed to Nand Async Timing Register 0
    NvU32 NandAsyncTiming0;

    /// Specifies the value to be programmed to Nand Async Timing Register 1
    NvU32 NandAsyncTiming1;

    /// Specifies the value to be programmed to Nand Async Timing Register 2
    NvU32 NandAsyncTiming2;

    /// Specifies the value to be programmed to Nand Async Timing Register 3
    NvU32 NandAsyncTiming3;

    /// Specifies the value to be programmed to Nand Sync DDR Timing Register 0
    NvU32 NandSDDRTiming0;

    /// Specifies the value to be programmed to Nand Sync DDR Timing Register 1
    NvU32 NandSDDRTiming1;

    /// Specifies the value to be programmed to Nand Toggle DDR Timing Register 0
    NvU32 NandTDDRTiming0;

    /// Specifies the value to be programmed to Nand Toggle DDR Timing Register 1
    NvU32 NandTDDRTiming1;

    /// Specifies the value to be programmed to FBIO_DQSIB_DELAY register
    NvU8 NandFbioDqsibDlyByte;

    /// Specifies the value to be programmed to FBIO_DQUSE_DELAY register
    NvU8 NandFbioQuseDlyByte;

    /// Specifies the CFG_QUSE_LATE value to be programmed to FBIO configuration register
    NvU8 NandFbioCfgQuseLate;

    /// Specifies whether to enable sync DDR more or not
    NvBool DisableSyncDDR;

    /// Specifies the block size in log2 bytes
    NvU8 BlockSizeLog2;

    /// Specifies the page size in log2 bytes
    NvU8 PageSizeLog2;


} NvBootNandParams;

#if defined(__cplusplus)
}
#endif

#endif /* #ifndef INCLUDED_NVBOOT_NAND_PARAM_H */
