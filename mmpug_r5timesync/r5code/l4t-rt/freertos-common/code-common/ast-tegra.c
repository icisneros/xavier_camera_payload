/*
 * Copyright (c) 2016-2017, NVIDIA CORPORATION.  All rights reserved.
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

#include <ast-tegra.h>
#include <ast-tegra-priv.h>

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <araps_ast.h>
#include <bitops.h>
#include <macros.h>
#include <nvrm_drf.h>
#include <reg-access.h>

#define LO_SHIFT APS_AST_REGION_0_SLAVE_BASE_LO_0_SlvBase_SHIFT

#define MAX_VMINDEX 15

/* Special values returned from VMINDEX search */
#define VMINDEX_PHYSICAL_STREAM_ID (MAX_VMINDEX + 1)
#define VMINDEX_INVALID (-1)
#define VMINDEX_NO_AST_BASE (-2)

static inline uint32_t ast_region_base(uint32_t ast_base, uint8_t region)
{
	const uint32_t region_stride =
		APS_AST_REGION_1_MASK_LO_0 - APS_AST_REGION_0_MASK_LO_0;

	return ast_base + region * region_stride;
}

static inline uint32_t streamid_ctl_reg(uint32_t ast_base, int vmindex)
{
	uint32_t ctl_stride = APS_AST_STREAMID_CTL_1 - APS_AST_STREAMID_CTL_0;

	return ast_base + APS_AST_STREAMID_CTL_0 + ((uint32_t)vmindex * ctl_stride);
}

static bool ast_is_global_locked(uint32_t ast_base)
{
	if (ast_base != 0U) {
		uint32_t control = readl(ast_base + APS_AST_CONTROL_0);

		return NV_DRF_VAL(APS_AST, CONTROL, Lock, control) != U32_C(0);
	} else {
		return false;
	}
}

bool tegra_ast_is_global_locked(const struct tegra_ast_id *id)
{
	return ast_is_global_locked(id->base_addr0)
		||
		ast_is_global_locked(id->base_addr1);
}

static uint8_t ast_get_physical_stream_id(uint32_t ast_base)
{
	uint32_t val = readl(ast_base + APS_AST_CONTROL_0);

	return NV_DRF_VAL(APS_AST, CONTROL, PhysStreamID, val);
}

static int ast_get_vmindex(uint32_t ast_base,
			uint8_t streamid,
			bool *return_is_enabled)
{
	uint32_t mask = APS_AST_STREAMID_CTL_0_READ_MASK;
	uint32_t enabled = NV_DRF_DEF(APS_AST, STREAMID_CTL, Enable, ENABLE);
	uint32_t expect = enabled |
		NV_DRF_NUM(APS_AST, STREAMID_CTL, StreamID, streamid);
	int free_vmindex = VMINDEX_INVALID;

	if (ast_base == 0U) {
		*return_is_enabled = true;
		return VMINDEX_NO_AST_BASE;
	}

	if (streamid == ast_get_physical_stream_id(ast_base)) {
		/*
		 * Global lock is used to prevent from configuring new
		 * regions with the Physical StreamID.
		 */
		if (!ast_is_global_locked(ast_base)) {
			*return_is_enabled = true;
			return VMINDEX_PHYSICAL_STREAM_ID;
		}
	}

	for (int vmindex = 0; vmindex <= MAX_VMINDEX; vmindex++) {
		uint32_t val = readl(streamid_ctl_reg(ast_base, vmindex)) & mask;

		if (val == expect) {
			*return_is_enabled = true;
			return vmindex;
		}

		/*
		 * Select last free vmindex in order to avoid
		 * collision w/ kernel and bootloader
		 */
		if ((val & enabled) != enabled) {
			free_vmindex = vmindex;
		}
	}

	*return_is_enabled = false;
	return free_vmindex;
}

static int ast_set_streamid(uint32_t ast_base,
			int vmindex,
			uint8_t streamid)
{
	uint32_t sid_ctl;
	uint32_t val;

	/* ast_get_vmindex ensures that ast_base != 0 */

	sid_ctl = NV_DRF_DEF(APS_AST, STREAMID_CTL, Enable, ENABLE) |
		NV_DRF_NUM(APS_AST, STREAMID_CTL, StreamID, streamid);

	writel(sid_ctl, streamid_ctl_reg(ast_base, vmindex));

	/* Check that write was not blocked */
	val = readl(streamid_ctl_reg(ast_base, vmindex));
	if ((val & APS_AST_STREAMID_CTL_READ_MASK) != sid_ctl) {
		return VMINDEX_INVALID;
	}

	return 0;
}

static int ast_add_streamid(uint32_t ast_base,
			uint8_t streamid)
{
	int vmindex;
	bool already_enabled;

	vmindex = ast_get_vmindex(ast_base, streamid, &already_enabled);
	if (vmindex < 0) {
		return vmindex;
	}

	if (already_enabled) {
		return 0;
	}

	return ast_set_streamid(ast_base, vmindex, streamid);
}

int tegra_ast_add_streamid(const struct tegra_ast_id *id,
			uint8_t streamid)
{
	if (ast_add_streamid(id->base_addr0, streamid) == VMINDEX_INVALID) {
		return -1;
	}

	if (ast_add_streamid(id->base_addr1, streamid) == VMINDEX_INVALID) {
		return -1;
	}

	return 0;
}

static int ast_set_physical_streamid(uint32_t ast_base,
			uint8_t streamid)
{
	uint32_t control;

	if (ast_base == 0U)
		return 0;

	control = readl(ast_base + APS_AST_CONTROL_0);

	if (streamid == NV_DRF_VAL(APS_AST, CONTROL, PhysStreamID, control))
		return 0;

	if (NV_DRF_VAL(APS_AST, CONTROL, Lock, control) != 0U)
		return -1;

	control = NV_FLD_SET_DRF_NUM(APS_AST, CONTROL, PhysStreamID, streamid,
				control);

	writel(control, ast_base + APS_AST_CONTROL_0);

	return 0;
}

int tegra_ast_set_physical_streamid(const struct tegra_ast_id *id,
			uint8_t streamid)
{
	if (ast_set_physical_streamid(id->base_addr0, streamid) < 0) {
		return -1;
	}

	if (ast_set_physical_streamid(id->base_addr1, streamid) < 0) {
		return -1;
	}

	return 0;
}

static void ast_enable_region(uint32_t ast_base,
				uint8_t region, int vmindex,
				uint64_t master, uint64_t slave,
				uint64_t size)
{
	uint32_t region_base = ast_region_base(ast_base, region);
	uint32_t control, hi, lo;

	if (ast_base == 0U)
		return;

	if (vmindex == VMINDEX_PHYSICAL_STREAM_ID) {
		control = NV_DRF_DEF(APS_AST, REGION_0_CONTROL, Snoop, ENABLE) |
			NV_DRF_NUM(APS_AST, REGION_0_CONTROL, Physical, 1U) |
			NV_DRF_NUM(APS_AST, REGION_0_CONTROL, VMIndex, 0U);
	} else {
		control = NV_DRF_DEF(APS_AST, REGION_0_CONTROL, Snoop, ENABLE) |
			NV_DRF_NUM(APS_AST, REGION_0_CONTROL, Physical, 0U) |
			NV_DRF_NUM(APS_AST, REGION_0_CONTROL, VMIndex,
				(uint32_t)vmindex);
	}

	writel(control, region_base + APS_AST_REGION_0_CONTROL_0);

	hi = (size - 1U) >> 32U;
	lo = ((size - 1U) & U64_C(0xFFFFffff)) >> LO_SHIFT;
	writel(NV_DRF_NUM(APS_AST, REGION_0_MASK_HI, Mask, hi),
		region_base + APS_AST_REGION_0_MASK_HI_0);
	writel(NV_DRF_NUM(APS_AST, REGION_0_MASK_LO, Mask, lo),
		region_base + APS_AST_REGION_0_MASK_LO_0);

	hi = master >> 32U;
	lo = (master & U64_C(0xFFFFffff)) >> LO_SHIFT;
	writel(NV_DRF_NUM(APS_AST, REGION_0_MASTER_BASE_HI, MastBase, hi),
		region_base + APS_AST_REGION_0_MASTER_BASE_HI_0);
	writel(NV_DRF_NUM(APS_AST, REGION_0_MASTER_BASE_LO, MastBase, lo),
		region_base + APS_AST_REGION_0_MASTER_BASE_LO_0);

	hi = slave >> 32U;
	lo = (slave & U64_C(0xFFFFffff)) >> LO_SHIFT;
	writel(NV_DRF_NUM(APS_AST, REGION_0_SLAVE_BASE_HI, SlvBase, hi),
		region_base + APS_AST_REGION_0_SLAVE_BASE_HI_0);
	writel(NV_DRF_NUM(APS_AST, REGION_0_SLAVE_BASE_LO, SlvBase, lo) |
		NV_DRF_DEF(APS_AST, REGION_0_SLAVE_BASE_LO, Enable, TRUE),
		region_base + APS_AST_REGION_0_SLAVE_BASE_LO_0);
}


int tegra_ast_enable_region(const struct tegra_ast_id *id,
				uint8_t region, uint8_t streamid,
				uint64_t master, uint64_t slave,
				uint64_t size)
{
	uint32_t base0 = id->base_addr0, base1 = id->base_addr1;
	bool already_enabled0, already_enabled1;
	int vmindex0, vmindex1;

	vmindex0 = ast_get_vmindex(base0, streamid, &already_enabled0);
	if (vmindex0 == VMINDEX_INVALID) {
		return -1;
	}

	vmindex1 = ast_get_vmindex(base1, streamid, &already_enabled1);
	if (vmindex1 == VMINDEX_INVALID) {
		return -1;
	}

	if (!already_enabled0) {
		if (ast_set_streamid(base0, vmindex0, streamid) == VMINDEX_INVALID) {
			return -1;
		}
	}

	if (!already_enabled1) {
		if (ast_set_streamid(base1, vmindex1, streamid) == VMINDEX_INVALID) {
			return -1;
		}
	}

	ast_enable_region(base0, region, vmindex0, master, slave, size);
	ast_enable_region(base1, region, vmindex1, master, slave, size);

	return 0;
}

void tegra_ast_disable_region(const struct tegra_ast_id *id, uint8_t region)
{
	uint32_t region_base;

	if (id->base_addr0 != 0) {
		region_base = ast_region_base(id->base_addr0, region);
		writel(NV_DRF_DEF(APS_AST, REGION_0_SLAVE_BASE_LO, Enable, FALSE),
			region_base + APS_AST_REGION_0_SLAVE_BASE_LO_0);
	}

	if (id->base_addr1 != 0) {
		region_base = ast_region_base(id->base_addr1, region);
		writel(NV_DRF_DEF(APS_AST, REGION_0_SLAVE_BASE_LO, Enable, FALSE),
			region_base + APS_AST_REGION_0_SLAVE_BASE_LO_0);
	}
}

static uint8_t ast_get_region_stream_id(uint32_t ast_base, uint8_t region)
{
	uint32_t region_base = ast_region_base(ast_base, region);
	uint32_t val, vmindex;

	if (ast_base == 0U) {
		return 0U;
	}

	val = readl(region_base + APS_AST_REGION_0_SLAVE_BASE_LO_0);

	if (!NV_DRF_VAL(APS_AST, REGION_0_SLAVE_BASE_LO, Enable, val)) {
		return 0U;
	}

	val = readl(region_base + APS_AST_REGION_0_CONTROL_0);

	if (NV_DRF_VAL(APS_AST, REGION_0_CONTROL, Physical, val)) {
		return ast_get_physical_stream_id(ast_base);
	}

	vmindex = NV_DRF_VAL(APS_AST, REGION_0_CONTROL, VMIndex, val);
	val = readl(streamid_ctl_reg(ast_base, vmindex));

	if (!NV_DRF_VAL(APS_AST, STREAMID_CTL, Enable, val)) {
		return 0U;
	}

	return NV_DRF_VAL(APS_AST, STREAMID_CTL, StreamID, val);
}

uint8_t tegra_ast_get_region_stream_id(
	const struct tegra_ast_id *id,
	uint8_t region)
{
	return ast_get_region_stream_id(id->base_addr0, region);
}

static bool ast_get_region_mapping(uint32_t ast_base, uint8_t region,
				struct tegra_ast_region_map *return_region_map)
{
	uint32_t region_base = ast_region_base(ast_base, region);
	uint32_t hi, lo;
	uint64_t mask;

	if (ast_base == 0U) {
		return false;
	}

	hi = readl(region_base + APS_AST_REGION_0_SLAVE_BASE_HI_0);
	lo = readl(region_base + APS_AST_REGION_0_SLAVE_BASE_LO_0);

	if ((lo & NV_DRF_DEF(APS_AST, REGION_0_SLAVE_BASE_LO, Enable, TRUE)) ==
		NV_DRF_DEF(APS_AST, REGION_0_SLAVE_BASE_LO, Enable, FALSE)) {
		return false;
	}

	if (return_region_map == NULL) {
		return true;
	}

	lo &= NV_DRF_DEF(APS_AST, REGION_0_SLAVE_BASE_LO, SlvBase, DEFAULT_MASK);

	return_region_map->slave_base = hilo_to_64(hi, lo);

	hi = readl(region_base + APS_AST_REGION_0_MASK_HI_0);
	lo = readl(region_base + APS_AST_REGION_0_MASK_LO_0) |
		~APS_AST_REGION_0_MASK_LO_0_READ_MASK;

	mask = hilo_to_64(hi, lo);
	return_region_map->size = mask + 1U;

	hi = readl(region_base + APS_AST_REGION_0_MASTER_BASE_HI_0);
	lo = readl(region_base + APS_AST_REGION_0_MASTER_BASE_LO_0);

	return_region_map->master_base = hilo_to_64(hi, lo);

	return_region_map->stream_id = ast_get_region_stream_id(ast_base, region);

	return true;
}

bool tegra_ast_get_region_mapping(
	const struct tegra_ast_id *id,
	uint8_t region,
	struct tegra_ast_region_map *return_region_map)
{
	if (return_region_map != NULL) {
		(void)memset(return_region_map, 0, sizeof(*return_region_map));
	}

	return ast_get_region_mapping(id->base_addr0, region,
					return_region_map);
}

uint64_t tegra_ast_map_pointer_to_iova(const struct tegra_ast_region_map *map,
			const void *pointer)
{
	return tegra_ast_map_slave_to_iova(map, (uint64_t)(uintptr_t)pointer);
}

void *tegra_ast_map_iova_to_pointer(const struct tegra_ast_region_map *map,
				uint64_t iova)
{
	return (void *)(uintptr_t)tegra_ast_map_iova_to_slave(map, iova);
}

uint64_t tegra_ast_map_slave_to_iova(const struct tegra_ast_region_map *map,
				uint64_t slave)
{
	uint64_t mask = map->size - 1U;

	if ((slave & ~mask) == map->slave_base) {
		return (slave & mask) | map->master_base;
	} else {
		return 0U;
	}
}

uint64_t tegra_ast_map_iova_to_slave(const struct tegra_ast_region_map *map,
				uint64_t iova)
{
	uint64_t mask = map->size - 1U;

	if ((iova & ~mask) == map->master_base) {
		return (iova & mask) | map->slave_base;
	} else {
		return 0U;
	}
}
