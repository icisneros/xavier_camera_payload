/*
 * Copyright (c) 2014 NVIDIA Corporation.  All rights reserved.
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
 * Defines fields in the PMC scratch registers used by the Boot ROM code.
 */

#ifndef INCLUDED_NVBOOT_PMC_SCRATCH_MAP_H
#define INCLUDED_NVBOOT_PMC_SCRATCH_MAP_H

/**
 * The PMC module in the Always On domain of the chip provides 43
 * scratch registers and 6 secure scratch registers. These offer SW a
 * storage space that preserves values across LP0 transitions.
 *
 * These are used by to store state information for on-chip controllers which
 * are to be restored during Warm Boot, whether WB0 or WB1.
 *
 * Scratch registers offsets are part of PMC HW specification - "arapbpm.spec".
 *
 * This header file defines the allocation of scratch register space for
 * storing data needed by Warm Boot.
 *
 * Each of the scratch registers has been sliced into bit fields to store
 * parameters for various on-chip controllers. Every bit field that needs to
 * be stored has a matching bit field in a scratch register. The width matches
 * the original bit fields.
 *
 * Scratch register fields have been defined with self explanatory names
 * and with bit ranges compatible with nvrm_drf macros.
 *
 * Ownership Issues: Important!!!!
 *
 * Register PMC_IMPL_SCRATCH0_0 is the *only* scratch register cleared on
 * power-on-reset. This register will also be used by RM and OAL. This holds
 * several important flags for the Boot ROM:
 *     WARM_BOOT0_FLAG: Tells the Boot ROM to perform WB0 upon reboot instead
 *                    of a cold boot.
 *     FORCE_RECOVERY_FLAG:
 *         Forces the Boot ROM to enter RCM instead of performing a cold
 *         boot or WB0.
 *     BL_FAIL_BACK_FLAG:
 *         One of several indicators that the the Boot ROM should fail back
 *         older generations of BLs if the newer generations fail to load.
 *     FUSE_ALIAS_FLAG:
 *         Indicates that the Boot ROM should alias the fuses with values
 *         stored in PMC SCRATCH registers and restart.
 *     STRAP_ALIAS_FLAG:
 *         Same as FUSE_ALIAS_FLAG, but for strap aliasing.
 *
 * The assignment of bit fields used by BootROM is *NOT* to be changed.
 */

/**
 * FUSE_ALIAS:
 *   Desc: Enables fuse aliasing code when set to 1.  In Pre-Production mode,
 *     the Boot ROM will alias the fuses and re-start the boot process using
 *     the new fuse values. Note that the Boot ROM clears this flag when it
 *     performs the aliasing to avoid falling into an infinite loop.
 *     Unlike AP15/AP16, the alias values are pulled from PMC scratch registers
 *     to survive an LP0 transition.
 * STRAP_ALIAS:
 *   Desc: Enables strap aliasing code when set to 1.  In Pre-Production mode,
 *     the Boot ROM will alias the straps and re-start the boot process using
 *     the new strap values. Note that the Boot ROM does not clear this flag
 *     when it aliases the straps, as this cannot lead to an infinite loop.
 *     Unlike AP15/AP16, the alias values are pulled from PMC scratch registers
 *     to survive an LP0 transition.
 */
#define SCRATCH_SCRATCH0_0_WARM_BOOT0_FLAG_RANGE                        0: 0
#define SCRATCH_SCRATCH0_0_FORCE_RECOVERY_FLAG_RANGE                    1: 1
#define SCRATCH_SCRATCH0_0_BL_FAIL_BACK_FLAG_RANGE                      2: 2
#define SCRATCH_SCRATCH0_0_BR_WDT_ENABLE_FLAG_RANGE                     3: 3
#define SCRATCH_SCRATCH0_0_BR_HALT_AT_WB0_FLAG_RANGE                    4: 4
// Bits 31:5 reserved for SW

// Likely needed for SW in same place
#define SCRATCH_SCRATCH1_0_PTR_TO_RECOVERY_CODE_RANGE                  31: 0

// TODO: Update LP0 exit PLLM registers for T35
#define SCRATCH_SCRATCH2_0_CLK_RST_CONTROLLER_PLLM_BASE_0_PLLM_DIVM_RANGE	 7: 0
#define SCRATCH_SCRATCH2_0_CLK_RST_CONTROLLER_PLLM_BASE_0_PLLM_DIVN_RANGE	15: 8
#define SCRATCH_SCRATCH2_0_CLK_RST_CONTROLLER_PLLM_BASE_0_PLLM_DIV2_RANGE	16:16
#define SCRATCH_SCRATCH2_0_CLK_RST_CONTROLLER_PLLM_MISC2_0_PLLM_KVCO_RANGE	17:17
#define SCRATCH_SCRATCH2_0_CLK_RST_CONTROLLER_PLLM_MISC2_0_PLLM_KCP_RANGE	19:18
// Bits 31:20 available

// Note: SCRATCH_SCRATCH3_0_CLK_RST_PLLX_CHOICE_RANGE identifies the choice
//       of PLL to start w/PLLX parameters: X or C.
// TODO: Update LP0 exit PLLX registers for T35
// Note: SCRATCH_SCRATCH3_0_CLK_RST_CONTROLLER_PLLX_ENABLE_RANGE need to
//          be set explicitly by BL/OS code in order to enable PLLX  in LP0 by
//          BR, by default PLLX is turned OFF.
#define SCRATCH_SCRATCH3_0_CLK_RST_CONTROLLER_PLLX_ENABLE_RANGE              23:23
// Bits 25:24 available
#define SCRATCH_SCRATCH3_0_CLK_RST_PLLX_CHOICE_RANGE                         26:26
// Bits 31:27 available

#define SCRATCH_SCRATCH4_0_PLLM_STABLE_TIME_RANGE                       9: 0
#define SCRATCH_SCRATCH4_0_PLLX_STABLE_TIME_RANGE                      19:10
// Bits 31:20 available

// PLLM extra params
// Bits 31 available
#define SCRATCH_SCRATCH35_0_CLK_RST_CONTROLLER_PLLM_MISC1_0_PLLM_PD_LSHIFT_PH135_RANGE 30:30
#define SCRATCH_SCRATCH35_0_CLK_RST_CONTROLLER_PLLM_MISC1_0_PLLM_PD_LSHIFT_PH90_RANGE  29:29
#define SCRATCH_SCRATCH35_0_CLK_RST_CONTROLLER_PLLM_MISC1_0_PLLM_PD_LSHIFT_PH45_RANGE  28:28
// Bits 27:24 available
#define SCRATCH_SCRATCH35_0_CLK_RST_CONTROLLER_PLLM_MISC1_0_PLLM_SETUP_RANGE 23: 0

// PLLX extra params
// Bits 31:24 available

// Storage for the location of the encrypted SE context
#define SCRATCH_SCRATCH43_0_SE_ENCRYPTED_CONTEXT_RANGE                 31: 0

// Storage location for bits that Boot ROM needs to restore at LP0.
#define SCRATCH_SCRATCH49_0_AHB_SPARE_REG_0_OBS_OVERRIDE_EN_RANGE        0:0
#define SCRATCH_SCRATCH49_0_AHB_SPARE_REG_0_APB2JTAG_OVERRIDE_EN_RANGE   1:1

// Storage for the SRK
/**
 * The SE will save the SRK key to SCRATCH4-7 when a CTX_SAVE operation with
 * destination SRK is started.
 */
#define SCRATCH_SECURE_SCRATCH4_0_SRK_0_SRK0_RANGE                     31: 0
#define SCRATCH_SECURE_SCRATCH5_0_SRK_0_SRK1_RANGE                     31: 0
#define SCRATCH_SECURE_SCRATCH6_0_SRK_0_SRK2_RANGE                     31: 0
#define SCRATCH_SECURE_SCRATCH7_0_SRK_0_SRK3_RANGE                     31: 0

#define SCRATCH_SCRATCH96_0_SE_NV_SECURE_GRP                           31:0
#define SCRATCH_SCRATCH97_0_PKA1_NV_SECURE_GRP                         31:0
#define SCRATCH_SCRATCH98_0_RNG1_NV_SECURE_GRP                         31:0

// Keep MC code drop at last. Use AS-IS
#include "nvboot_wb0_sdram_scratch_list.h"

#endif // INCLUDED_NVBOOT_PMC_SCRATCH_MAP_H
