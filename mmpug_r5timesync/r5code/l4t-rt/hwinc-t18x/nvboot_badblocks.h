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
 * @file
 * <b>NVIDIA Tegra ODM Kit:
 *         Bad Blocks Table (Tegra APX)</b>
 *
 * @b Description: NvBootBadBlockTable contains a description of the known
 * bad blocks, which are skipped over when loading BCTs and boot loaders.
 */

/**
 * @defgroup nvbl_bad_block_group Bad Blocks Table (Tegra APX)
 * @ingroup nvbl_bct_group
 * @{
 *
 * To keep the size of the table manageable, the concept of virtual blocks
 * is used. The \c BadBlocks[] array represents the state of each of the
 * virtual blocks with a single bit that is a '1' if the block is known
 * to be bad.
 *
 * The size of a virtual block must be >= the actual block size
 * of the device. If there is enough space in the \c BadBlocks[] array,
 * the virtual block size should be equal to the actual block size.
 * However, the virtual block size can be increased to ensure that
 * it covers the portion of the secondary boot device read by the
 * boot ROM. The tradeoff is coarser granularity of bad block information.
 *
 * If the boot ROM attempts to read from a block outside the range of the
 * table, it presumes the block to be good.
 *
 * Given a block number, its status can be given as:
 * <pre>
 *   VirtualBlock = Block >> (Table->BlockSizeLog2 -
 *                            Table->VirtualBlockSizeLog2);
 *   TableEntry   = Table->BadBlocks[VirtualBlock >> 3];
 *   Status       = TableEntry & (1 << (VirtualBlock & 0x7));
 * </pre>
 */

#ifndef INCLUDED_NVBOOT_BADBLOCKS_H
#define INCLUDED_NVBOOT_BADBLOCKS_H

#include "nvcommon.h"
#include "nvboot_config.h"

#if defined(__cplusplus)
extern "C"
{
#endif


/**
 * Defines the bad block table structure stored in the BCT.
 */
typedef struct NvBootBadBlockTableRec
{
    /// Specifies the number of actually used entries in the bad block table.
    NvU32 EntriesUsed;

    /// Specifies the size of a virtual block as log2(# of bytes).
    /// This must be >= the physical block size.
    NvU8  VirtualBlockSizeLog2;

    /// Specifies the actual size of a block as log2(# of bytes).
    NvU8  BlockSizeLog2;

    /// Specifies the state of each virtual block with a single bit.
    /// A '1' bit indicates that the corresponding region in the storage
    /// device is known to be bad.
    NvU8  BadBlocks[NVBOOT_BAD_BLOCK_TABLE_SIZE / 8];

    /// Add a reserved field as padding to make the bad block table structure
    /// a multiple of 16 bytes (AES block size).
    NvU8  Reserved[NVBOOT_BAD_BLOCK_TABLE_PADDING];
} NvBootBadBlockTable;

#if defined(__cplusplus)
}
#endif

/** @} */
#endif /* #ifndef INCLUDED_NVBOOT_BADBLOCKS_H */
