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
 * Defines the warm boot 0 information for the boot rom.
 */

#ifndef INCLUDED_NVBOOT_WARM_BOOT_0_H
#define INCLUDED_NVBOOT_WARM_BOOT_0_H

#include "nvboot_config.h"
#include "nvboot_crypto_param.h"
#include "nvboot_config.h"
#include "nvboot_fuse.h"
#include "nvboot_hash.h"


#if defined(__cplusplus)
extern "C"
{
#endif

/**
 * Defines the recovery code header information for the boot rom.
 *
 * The recovery code immediately follows the recovery code header.
 *
 * Note that the recovery code header needs to be 16 bytes aligned to preserve
 * the alignment of relevant data for hash and decryption computations without
 * requiring extra copies to temporary memory areas.
 */
typedef struct NvBootWb0RecoveryHeaderRec
{
    /// Specifies the length of the recovery code header
    NvU32      LengthInsecure;

    /// Specifies the reserved words to maintain alignment
    NvU32      Reserved[3];
    
    /// This header will house public, non-secret cryptographic parameters necessary
    /// for the authentication of the BCT and Boot Images. These parameters are
    /// collectively known as Public Cryptographic Parameters (PCP) and they will
    /// be stored in the unsigned section of the BCT.
    /// The BR will check the validity of these parameters by calculating the SHA256
    /// hash of the Pcp and compare against the value burned in fuses.
    NvBootPublicCryptoParameters Pcp;

    /// All cryptographic signatures supported will be stored here. The BCT can be
    /// simultaneously signed by all cryptographic signature types.
    NvBootCryptoSignatures Signatures;

    /// Specifies the random block of data which is not validated but
    /// aids security.
    NvBootHash RandomAesBlock;

    /// Specifies the length of the recovery code header
    NvU32      LengthSecure;

    /// Specifies the starting address of the recovery code in the
    /// destination area.
    NvU32      Destination;

    /// Specifies the entry point of the recovery code in the destination area.
    NvU32      EntryPoint;

    /// Specifies the length of the recovery code
    NvU32      RecoveryCodeLength;
} NvBootWb0RecoveryHeader;

#if defined(__cplusplus)
}
#endif

#endif /* #ifndef INCLUDED_NVBOOT_WARM_BOOT_0_H */
