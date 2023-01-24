/*
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
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

#include <address_map_new.h>
#include <macros.h>
#include <irqs-hw.h>
#include <irqs-lic.h>

#define LIC_IRQ_MAPPING(aon, lic) [LIC_IRQ_OFFSET(aon)] = TEGRA_LIC_BITSLICE(lic)
#define LIC_IRQ_OFFSET(aon) (AON_LIC_IRQ_ ## aon - AON_LIC_IRQ_BASE)

const struct tegra_lic_bitslice lic_mapping[] = {
	LIC_IRQ_MAPPING(TOP_HSP0, TOP_HSP0_SHARED_0),
};

const struct tegra_lic_id tegra_lic_id_aon = {
	.base_channel = 2,
	.num_channels = 2,
	.num_slices = 9,
	.local_irq = NV_AON_INTERRUPT_LIC0,
	.lic_irq_base = AON_LIC_IRQ_BASE,
	.lic_map_size = ARRAY_SIZE(lic_mapping),
	.lic_map = lic_mapping,
};
