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
#include <clk-tegra-hw.h>
#include <arclk_rst.h>

#include <tegra-mttcan.h>
#include <irqs-hw.h>

struct ttcan_controller tegra_gttcan[] = {
	[0] = {
		.base = NV_ADDRESS_MAP_CAN1_BASE,
		.xbase = NV_ADDRESS_MAP_CAN1_BASE + CAN_GLUE_ADDR,
		.rst = tegra_rst_can0,
		.mram_sa.virt_base = NV_ADDRESS_MAP_CAN1_BASE + CAN_MES_RAM_BASE_ADDR,
		.mram_sa.base =  NV_ADDRESS_MAP_CAN1_BASE + CAN_MES_RAM_BASE_ADDR,
		.id = 0,
		.irq = NV_AON_INTERRUPT_VIC0_BASE + NV_AON_INTERRUPT_CAN1_0,
		.data = 0, /* Bit-0 defines interrupt line, Bit-1 defines controller */
	},
	[1] = {
		.base = NV_ADDRESS_MAP_CAN2_BASE,
		.xbase = NV_ADDRESS_MAP_CAN2_BASE + CAN_GLUE_ADDR,
		.rst = tegra_rst_can1,
		.mram_sa.virt_base = NV_ADDRESS_MAP_CAN2_BASE + CAN_MES_RAM_BASE_ADDR,
		.mram_sa.base =  NV_ADDRESS_MAP_CAN2_BASE + CAN_MES_RAM_BASE_ADDR,
		.id = 1,
		.irq = NV_AON_INTERRUPT_VIC0_BASE + NV_AON_INTERRUPT_CAN2_0,
		.data = 2, /* Bit-0 defines interrupt line, Bit-1 defines controller */
	}
};
