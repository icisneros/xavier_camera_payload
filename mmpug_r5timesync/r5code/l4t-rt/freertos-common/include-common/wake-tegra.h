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

#ifndef _WAKE_TEGRA_H_
#define _WAKE_TEGRA_H_

#include <stdbool.h>
#include <stdint.h>

struct tegra_wake_id;

/*
 * To use this driver:
 *
 * 1. Call tegra_wake_route_event to route wake event into desired tier level
 * 2. Call tegra_wake_enable_irq to enable the irq of the selected wake event
 * 2. Call tegra_wake_enable to enable wake event detection
 */

/*
 * Route wake event into selected tier
 *
 * Parameters:
 * id:		The ID structure for the WAKE module
 * wake_event:	The wake event to be routed
 * tier:	The wake event will be routed into this tier
 * sel:		0: Remove wake event from desired tier level
 * 		1: Select wake event into desired tier level
 */
void tegra_wake_route_event(struct tegra_wake_id *id, uint32_t wake_event,
				  uint32_t tier, bool sel);

/*
 * Enable selected wake event
 *
 * Parameters:
 * id:		The ID structure for the WAKE module
 * wake_event:	The wake event is going to be enabled
 */
void tegra_wake_enable_event(struct tegra_wake_id *id, uint32_t wake_event);

/*
 * Disable selected wake event
 *
 * Parameters:
 * id:		The ID structure for the WAKE module
 * wake_event:	The wake event is going to be disabled
 */
void tegra_wake_disable_event(struct tegra_wake_id *id, uint32_t wake_event);

/*
 * Enable wake event detection
 *
 * Parameters:
 * id:		The ID structure for the WAKE module
 */
void tegra_wake_enable(struct tegra_wake_id *id);

/*
 * Disable wake event detection
 *
 * Parameters:
 * id:		The ID structure for the WAKE module
 */
void tegra_wake_disable(struct tegra_wake_id *id);

/*
 * Clear wake interrupt status
 *
 * Parameters:
 * id:		The ID structure for the WAKE module.
 * wake_event:	The wake event status to be clear
 */
void tegra_wake_clear_irq(struct tegra_wake_id *id, uint32_t wake_event);

/*
 * Trigger a wake event
 *
 * Parameters:
 * id:		The ID structure for the WAKE module.
 */
void tegra_wake_trigger_wake_event(struct tegra_wake_id *id);

#endif
