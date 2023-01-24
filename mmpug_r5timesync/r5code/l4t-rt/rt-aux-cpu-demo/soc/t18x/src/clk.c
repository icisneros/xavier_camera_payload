/* Copyright (c) 2017, NVIDIA CORPORATION. All rights reserved.
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

#include <FreeRTOS.h>

#include <address_map_new.h>
#include <arclk_rst.h>
#include <araopm.h>
#include <clk-tegra.h>
#include <delay.h>
#include <nvrm_drf.h>
#include <reg-access.h>

#include <clk.h>

void clk_init_hw(void)
{
	uint32_t val;

	/* Disable burst clk change on IRQ */
	writel(0, (NV_ADDRESS_MAP_AON_PM_BASE + AOPM_BURST_CLK_0));

	/* Enables turning on PLLAON in SC7 */
	val = readl(NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLAON_MISC_4_0);
	val = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, PLLAON_MISC_4, PLLAON_SEL_IREF,
			1, val);
	writel(val, NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_PLLAON_MISC_4_0);

	udelay(5);

	/* Program the aon_cpu_nic_clk to run off the pllaon with div=2 @ 240Mhz */
	val = NV_DRF_DEF(CLK_RST_CONTROLLER,
			CLK_SOURCE_AON_CPU_NIC,
			AON_CPU_NIC_CLK_SRC, pllAON_out) |
		NV_DRF_NUM(CLK_RST_CONTROLLER,
			CLK_SOURCE_AON_CPU_NIC,
			AON_CPU_NIC_CLK_DIVISOR, 2);

	writel(val, NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_CLK_SOURCE_AON_CPU_NIC_0);

	/* Program the aon_apb_clk to run off the pllaon @ 120Mhz */
	val = NV_DRF_DEF(CLK_RST_CONTROLLER,
			CLK_SOURCE_AON_APB,
			AON_APB_CLK_SRC, pllAON_out) |
		NV_DRF_NUM(CLK_RST_CONTROLLER,
			CLK_SOURCE_AON_APB,
			AON_APB_CLK_DIVISOR, 6);

	writel(val, NV_ADDRESS_MAP_CAR_BASE + CLK_RST_CONTROLLER_CLK_SOURCE_AON_APB_0);
}
