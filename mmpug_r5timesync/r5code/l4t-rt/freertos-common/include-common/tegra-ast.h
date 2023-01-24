/*
 * Copyright (c) 2015, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _TEGRA_AST_H_
#define _TEGRA_AST_H_

#include <stdint.h>

/*
 * I believe the typical use-case is that the AST is configured and locked by
 * system boot software, and the auxilliary processors simply use that
 * configuration without modification. Hence, an AST driver should not be
 * required. However, some simulation environments (HW tree running linsim
 * without ASIM, SCE unit FPGA with SCE code download via JTAG) have no CCPLEX
 * to configure the AST, and hence we need a minimal driver for those cases.
 */

/*
 * Perform a simple initialization of the AST, enabling just VMIndex 0.
 */
void tegra_ast_init(uint32_t ast_base);

/*
 * Configure an AST region's memory mapping attributes, with hard-coded
 * control parameters (VMIndex 0, non-physical, non-secure, etc.)
 */
void tegra_ast_region_config_enable(uint32_t ast_base, int region,
	uint32_t slave_base, uint32_t mask, uint64_t master_base);

/*
 * Disable an AST region.
 */
void tegra_ast_region_disable(uint32_t ast_base, int region);

/*
 * Set the stream ID of a given VM index.
 */
void tegra_ast_set_streamid(uint32_t ast_base, int vmindex, int streamid);
#endif
