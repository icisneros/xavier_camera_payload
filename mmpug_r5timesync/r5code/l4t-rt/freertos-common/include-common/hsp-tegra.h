/*
 * Copyright (c) 2015-2016 NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _HSP_TEGRA_H
#define _HSP_TEGRA_H

#include <stdint.h>
#include <stdbool.h>

#include <FreeRTOS.h>

struct tegra_hsp_id;

struct tegra_hsp_suspend_ctx {
	uint32_t db_enable;
	uint32_t si_enable[8];
};

typedef void (*tegra_hsp_callback)(uint32_t source, BaseType_t *higher_prio_task_woken);

int tegra_hsp_db_init(const struct tegra_hsp_id *id, uint32_t enabled_masters, tegra_hsp_callback callback);
void tegra_hsp_db_ring(const struct tegra_hsp_id *id, uint32_t target);
void tegra_hsp_db_enable_master(const struct tegra_hsp_id *id, uint32_t enabled_masters);
void tegra_hsp_db_disable_master(const struct tegra_hsp_id *id, uint32_t disabled_masters);
void tegra_hsp_db_irq_handler(void *id);

int tegra_hsp_suspend(const struct tegra_hsp_id *id, struct tegra_hsp_suspend_ctx *ctx);
int tegra_hsp_resume(const struct tegra_hsp_id *id, struct tegra_hsp_suspend_ctx *ctx);

void tegra_hsp_init(const struct tegra_hsp_id *id);
void tegra_hsp_sm_full_enable(const struct tegra_hsp_id *id, uint32_t sm,
			void (*)(void *, uint32_t), void *);
void tegra_hsp_sm_full_disable(const struct tegra_hsp_id *id, uint32_t sm);
void tegra_hsp_sm_empty_enable(const struct tegra_hsp_id *id, uint32_t sm,
			void (*cb)(void *, uint32_t), void *data);
void tegra_hsp_sm_empty_disable(const struct tegra_hsp_id *id, uint32_t sm);
void tegra_hsp_sm_produce(const struct tegra_hsp_id *id, uint32_t sm, int32_t value);
int32_t tegra_hsp_sm_consume(const struct tegra_hsp_id *id, uint32_t sm);
int32_t tegra_hsp_sm_peek(const struct tegra_hsp_id *id, uint32_t sm);
void tegra_hsp_sm_vacate(const struct tegra_hsp_id *id, uint32_t sm);
bool tegra_hsp_sm_is_empty(const struct tegra_hsp_id *id, uint32_t sm);

/* The shared semaphores are registers with associated set/clr addresses
 * to allow easy manipulation of individual bits inside them i.e. without
 * the need of RMW operation. There is no HW arbitration. So, SW must statically
 * allocate the ownership of individual semaphores for correct operation.
 * Summary of shared semaphore registers:
 *
 *   Name	   Offset	Type	Description
 * SHRD_SMP_STA	   0x000	RO	Current value
 * SHRD_SMP_SET	   0x004	WO	Bits to set in current value. Writing
 *					bit i to 1 sets semaphore bit i.
 * SHRD_SMP_CLR	   0x008	WO	Bits to clear in current value. Writing
 *					bit i to 1 clears semaphore bit i.
 */

/*
 * Reads the current value of the shared semaphore specified by
 * the index.
 *
 * Parameters:
 * index:	The shared semaphore index whose value has to be read.
 * value:	A pointer to where the read value should be returned in.
 *
 * Returns:
 *  0: Success
 * -1: Failure
 */
int tegra_hsp_ss_read(const struct tegra_hsp_id *id, uint32_t index, uint32_t *value);

/*
 * Sets the bits specified in the data parameter in the shared semaphore.
 *
 * Parameters:
 * index:	The shared semaphore index to which data has to be written.
 * data:	Specify the bits to be set in the semaphore.
 *
 * Returns:
 *  0: Success
 * -1: Failure
 */
int tegra_hsp_ss_set(const struct tegra_hsp_id *id, uint32_t index, uint32_t data);

/*
 * Clears the bits specified in the data parameter in the shared semaphore.
 *
 * Parameters:
 * index:	The shared semaphore index to which data has to be written.
 * data:	Specify the bits to be cleared in the semaphore.
 *
 * Returns:
 *  0: Success
 * -1: Failure
 */
int tegra_hsp_ss_clear(const struct tegra_hsp_id *id, uint32_t index, uint32_t data);

#endif
