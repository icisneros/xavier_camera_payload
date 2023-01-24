/*
 * Copyright (c) 2006-2009, NVIDIA CORPORATION. All rights reserved.
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
 * @file nvboot_aes.h
 *
 * Defines the parameters and data structure for AES.
 *
 * AES is used for encryption, decryption, and signatures.
 */

#ifndef INCLUDED_NVBOOT_AES_H
#define INCLUDED_NVBOOT_AES_H

#include "nvcommon.h"
#include "nvboot_config.h"

#if defined(__cplusplus)
extern "C"
{
#endif

/**
 * Defines AES Engine instances.
 */
typedef enum
{
    /// Specifies AES Engine "A" (in BSEV).
    NvBootAesEngine_A,

    /// Specifies AES Engine "B" (in BSEA).
    NvBootAesEngine_B,

    NvBootAesEngine_Num,
    NvBootAesEngine_Force32 = 0x7FFFFFFF
} NvBootAesEngine;

/**
 * Defines AES Key Slot instances (per AES engine).
 */
typedef enum
{
    /// Specifies AES Key Slot "0"
    //  This is an insecure slot.
    NvBootAesKeySlot_0,

    /// Specifies AES Key Slot "1"
    //  This is an insecure slot.
    NvBootAesKeySlot_1,

    /// Specifies AES Key Slot "2"
    //  This is an insecure slot.
    NvBootAesKeySlot_2,

    /// Specifies AES Key Slot "3"
    //  This is an insecure slot.
    NvBootAesKeySlot_3,

    /// Specifies AES Key Slot "4"
    //  This is a secure slot.
    NvBootAesKeySlot_4,

    /// Specifies AES Key Slot "5"
    //  This is a secure slot.
    NvBootAesKeySlot_5,

    /// Specifies AES Key Slot "6"
    //  This is a secure slot.
    NvBootAesKeySlot_6,

    /// Specifies AES Key Slot "7"
    //  This is a secure slot.
    NvBootAesKeySlot_7,

    NvBootAesKeySlot_Num,

    /// Specifies the AES Key Schedule Enable
    NvBootAesKeySlot_SchedEnb,

    NvBootAesKeySlot_Force32 = 0x7FFFFFFF
} NvBootAesKeySlot;


/**
 * Defines the maximum length of an Initial Vector (IV) in units of
 * 32 bit words.
 */
enum {NVBOOT_AES_MAX_IV_LENGTH = 8};

/**
 * Defines the length of an Initial Vector (IV) as used by the Boot ROM
 * in units of 32 bit words.
 */
enum {NVBOOT_AES_IV_LENGTH = 4};

/**
 * Defines the length of a key in units of 32 bit words.
 */
enum {NVBOOT_AES_KEY_LENGTH = 4};

/**
 * Defines the length of a key in units of bytes
 */
enum {NVBOOT_AES_KEY_LENGTH_BYTES = 16};

/**
 * Defines the length of an AES block in units of 32 bit words.
 */
enum {NVBOOT_AES_BLOCK_LENGTH = 4};

/**
 * Defines the length of an AES block in units of log2 bytes.
 */
enum {NVBOOT_AES_BLOCK_LENGTH_LOG2 = 4};


/**
 * Defines an AES Key (128 bits).
 */
typedef struct NvBootAesKeyRec
{
    /// Specifies the key data.
    NvU32 key[NVBOOT_AES_KEY_LENGTH];
} NvBootAesKey;

/**
 * Defines an AES Initial Vector (128 bits).
 */
typedef struct NvBootAesIvRec
{
    /// Specifies the initial vector data.
    NvU32 iv[NVBOOT_AES_IV_LENGTH];
} NvBootAesIv;

#if defined(__cplusplus)
}
#endif

#endif // INCLUDED_NVBOOT_AES_H
