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

#ifndef INCLUDE_AST_TEGRA_H
#define INCLUDE_AST_TEGRA_H

#include <stdint.h>
#include <stdbool.h>

struct tegra_ast_id;

struct tegra_ast_region_map {
	uint64_t master_base;
	uint64_t slave_base;
	uint64_t size;
	uint8_t stream_id;
};

/**
 * Map memory via an AST region.
 *
 * @param[in] id id structure describing the ast resource
 * @param region region number (0..7)
 * @param stream_id StreamID used with region
 * @param master_base base address in private address space
 * @param slave_base base address in SoC address space
 * @param size size of the region
 *
 * The @s size of the region must be power of 2. Regions may not
 * overlap.
 *
 * If the @a stream_id is not programmed in the ast, it is added to
 * the list of stream_ids, unless the list is locked by bootloader.
 *
 * @retval 0 success
 * @retval -1 failure
 */
int tegra_ast_enable_region(const struct tegra_ast_id *id,
				uint8_t region, uint8_t stream_id,
				uint64_t master_base, uint64_t slave_base,
				uint64_t size);

/**
 * Disable an AST region.
 *
 * @param[in] id id structure describing ast resource
 * @param region region number (0..7)
 */
void tegra_ast_disable_region(const struct tegra_ast_id *id,
			uint8_t region);

/**
 * Get the AST mapping for region.
 *
 * @param[in] id id structure describing the ast resource
 * @param region region number (0..7)
 * @param[out] return_region_map return value for region mapping
 *              (may be NULL)
 *
 * @retval false if region is not enabled.
 * @retval true if region is enabled.
 */
bool tegra_ast_get_region_mapping(const struct tegra_ast_id *id,
		uint8_t region,
		struct tegra_ast_region_map *return_region_map);

/**
 * Get the StreamID used with for region.
 * @param[in] id id structure describing the ast resource
 * @param region region number (0..7)
 *
 * @retval 0 if region is not enabled or has no valid StreamID
 * @return StreamdID
 */
uint8_t tegra_ast_get_region_stream_id(const struct tegra_ast_id *id,
				uint8_t region);

/**
 * Add and enable a StreamID.
 *
 * @param[in] id id structure describing the ast resource
 * @param stream_id StreamID to add
 *
 * @retval 0 success
 * @retval -1 failure
 */
int tegra_ast_add_streamid(const struct tegra_ast_id *id,
			uint8_t streamid);

/**
 * Check if AST is globally locked.
 *
 * It is not possible to change the Physical StreamID or define new
 * regions with Physical StreamID if AST is globally locked.
 *
 * @param[in] id id structure describing the ast resource
 */
bool tegra_ast_is_global_locked(const struct tegra_ast_id *id);

/**
 * Set the Physical StreamID.
 *
 * @param[in] id id structure describing the ast resource
 * @param stream_id Physical StreamID to add
 *
 * @retval 0 success
 * @retval -1 failure
 */
int tegra_ast_set_physical_streamid(const struct tegra_ast_id *id,
				uint8_t streamid);

/**
 * Map pointer to IOVA in ast region.
 *
 * @param map region mapping
 * @param pointer pointer in slave address space
 *
 * @return IOVA or 0 if address is not inside mapping.
 */
uint64_t tegra_ast_map_pointer_to_iova(const struct tegra_ast_region_map *map,
				const void *pointer);

/** Map IOVA to pointer in ast region.
 *
 * @param map region mapping
 * @param iova 64-bit I/O virtual address
 *
 * @return A pointer corresponding to @a iova or NULL if it is not inside
 * mapping.
 */
void *tegra_ast_map_iova_to_pointer(const struct tegra_ast_region_map *map,
				uint64_t iova);

/**
 * Map slave address to IOVA in ast region.
 *
 * @param map region mapping
 * @param slave slave address
 *
 * @return IOVA or 0 if @slave is not inside mapping.
 */
uint64_t tegra_ast_map_slave_to_iova(const struct tegra_ast_region_map *map,
				uint64_t slave);

/** Map IOVA to slave address in ast region.
 *
 * @param map region mapping
 * @param iova 64-bit I/O virtual address
 *
 * @return A slave address corresponding to @a iova or 0 if it is not
 * inside mapping.
 */
uint64_t tegra_ast_map_iova_to_slave(const struct tegra_ast_region_map *map,
				uint64_t iova);

#endif /* INCLUDE_AST_TEGRA_H */
