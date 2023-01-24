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

#include <araps_ast.h>
#include <nvrm_drf.h>

#include <reg-access.h>
#include <tegra-ast.h>

/*
 * This is some arbitrarily picked stream ID value that we use. We assume this
 * ID is mapped 1:1 with physical memory, or irrelevant since the SMMU is
 * disabled. This value may need to be adjusted as simulation environments
 * change. If this driver is ever used for "real-world" use-cases, we'll need
 * to replace this constant and all of tegra_ast_init() with something much
 * more parameterized/flexible.
 */
#define ARBITRARY_STREAMID 0x7f

static inline uint32_t region_offset(int region)
{
	const uint32_t region_stride =
		APS_AST_REGION_1_MASK_LO_0 - APS_AST_REGION_0_MASK_LO_0;

	return region * region_stride;
}

static inline uint32_t page_no(uint32_t addr)
{
	const uint32_t page_shift = 12;

	return addr >> page_shift;
}

void tegra_ast_init(uint32_t ast_base)
{
	int i;

	writel(NV_DRF_DEF(APS_AST, CONTROL, MatchErrCtl, DECERR),
		ast_base + APS_AST_CONTROL_0
	);

	writel(NV_DRF_NUM(APS_AST, STREAMID_CTL, StreamID, ARBITRARY_STREAMID) |
		NV_DRF_DEF(APS_AST, STREAMID_CTL, Enable, ENABLE),
		ast_base + APS_AST_STREAMID_CTL_0
	);

	for (i = 1; i < 8; i++) {
		writel(NV_DRF_DEF(APS_AST, STREAMID_CTL, Enable, DISABLE),
			ast_base + APS_AST_STREAMID_CTL_0 + (i * 4)
		);
	}
}

void tegra_ast_region_config_enable(uint32_t ast_base, int region,
	uint32_t slave_base, uint32_t mask, uint64_t master_base)
{
	uint32_t roffset = region_offset(region);

	writel(0, ast_base + APS_AST_REGION_0_MASK_HI_0 + roffset);

	writel(NV_DRF_NUM(APS_AST, REGION_0_MASK_LO, Mask, page_no(mask)),
		ast_base + APS_AST_REGION_0_MASK_LO_0 + roffset
	);

	writel(0, ast_base + APS_AST_REGION_0_MASTER_BASE_HI_0 + roffset);

	writel(NV_DRF_NUM(APS_AST, REGION_0_MASTER_BASE_LO, MastBase,
		page_no(master_base)),
		ast_base + APS_AST_REGION_0_MASTER_BASE_LO_0 + roffset
	);

	writel(NV_DRF_DEF(APS_AST, REGION_0_CONTROL, Snoop, ENABLE),
		ast_base + APS_AST_REGION_0_CONTROL_0 + roffset
	);

	writel(0, ast_base + APS_AST_REGION_0_SLAVE_BASE_HI_0 + roffset);

	writel(NV_DRF_NUM(APS_AST, REGION_0_SLAVE_BASE_LO, SlvBase,
		page_no(slave_base)) |
		NV_DRF_DEF(APS_AST, REGION_0_SLAVE_BASE_LO, Enable, TRUE),
		ast_base + APS_AST_REGION_0_SLAVE_BASE_LO_0 + roffset
	);
}

void tegra_ast_region_disable(uint32_t ast_base, int region)
{
	uint32_t roffset = region_offset(region);

	writel(NV_DRF_DEF(APS_AST, REGION_0_SLAVE_BASE_LO, Enable, FALSE),
		ast_base + APS_AST_REGION_0_SLAVE_BASE_LO_0 + roffset
	);
}

void tegra_ast_set_streamid(uint32_t ast_base, int vmindex, int streamid)
{
	writel(NV_DRF_NUM(APS_AST, STREAMID_CTL, StreamID, streamid) |
		NV_DRF_DEF(APS_AST, STREAMID_CTL, Enable, ENABLE),
		ast_base + APS_AST_STREAMID_CTL_0 + (vmindex * 4)
	);
}
