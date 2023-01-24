/*
 * Copyright (c) 2015-2017, NVIDIA CORPORATION.  All rights reserved.
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

#include <stdio.h>
#include <string.h>

#include <address_map_new.h>

#include <ast-tegra.h>
#include <ast-tegra-hw.h>
#include <barriers.h>
#include <cache.h>
#include <err-hook.h>
#include <printf-isr.h>
#include <tegra-ivc.h>
#include <hsp-tegra.h>
#include <hsp-tegra-hw.h>
#include <spi-tegra-hw.h>
#include <macros.h>
#include <printf-isr.h>

#include <clk.h>
#include <ivc-config.h>
#include <ivc-channels.h>
#include <mbox-aon.h>
#include <dbgprintf.h>

static bool ast_inited;
static void *ccplex_task_states[IVC_NUM_CCPLEX_CHS];

static int ivc_init_channels_from_ram_ccplex(BaseType_t *higher_prio_task_woken)
{
	int i, ret = 0;

	for (i = 0; i < IVC_NUM_CCPLEX_CHS; i++) {
		if (tegra_ivc_init_channel_from_ram(ivc_ccplex_task_ids[i]->ivc_ch)) {
			error_hookf("tegra_ivc_init_channel_from_ram(%s) failed",
					ivc_ccplex_task_ids[i]->name);
			ret = -1;
			continue;
		}
		if (ccplex_task_states[i] && ivc_ccplex_task_ids[i]->ops->init_complete)
			ivc_ccplex_task_ids[i]->ops->init_complete(ccplex_task_states[i],
					higher_prio_task_woken);
	}

	return ret;
}

static int ivc_channels_ivc_notified_ccplex_ns(BaseType_t *higher_prio_task_woken)
{
	int ret = -1;
	int i;

	for (i = 0; i < IVC_NUM_CCPLEX_CHS; i++)
		if (ccplex_task_states[i] &&
				ivc_ccplex_task_ids[i]->ops->notify)
			ret = ivc_ccplex_task_ids[i]->ops->notify(
					ccplex_task_states[i],
					higher_prio_task_woken);

	return ret;
}

static int ivc_carveout_ast_setup(uint32_t phys_addr, uint32_t carveout_size)
{
	if (carveout_size & (carveout_size - 1)) {
		error_hookf("Carveout size not a power of 2: %lx", carveout_size);
		return -1;
	}

	if (carveout_size < AST_MIN_CARVEOUT_SIZE) {
		error_hookf("Carveout size is too small: %lx", carveout_size);
		return -1;
	}

	if (phys_addr & (carveout_size - 1)) {
		error_hookf("Carveout is not aligned: %lx/%lx", phys_addr, carveout_size);
		return -1;
	}

	/*
	 * The bootloader is responsible for initializing:
	 *	- AST region 0 to point at SPE SYSRAM carve-out.
	 *	- AST region 1 to point at SPE GSC DRAM carve-out.
	 * The SPE firmware is responsible for initializing:
	 *	- AST region 2 to point at the IVC carve-out in DRAM.
	 */
	if (tegra_ast_enable_region(&tegra_ast_id_aon, AST_IVC_REGION,
			AON_STREAMID, phys_addr, CCPLEX_CARVEOUT_BASE,
			carveout_size)) {
		error_hook("AST init failed");
		return -1;
	}

	barrier_memory_complete();

	return 0;
}

static void ivc_ccplex_carveout_configure(BaseType_t *higher_prio_task_woken)
{
	static int count = 0;
	uint32_t carveout_base;
	uint32_t carveout_size;
	int ret;

	count++;
	if (count > 1)
		return;
	ret = tegra_hsp_ss_read(&tegra_hsp_id_aon, IVC_CARVEOUT_BASE_SS_INDEX,
				&carveout_base);
	if (ret)
		goto err;
	tegra_hsp_ss_clear(&tegra_hsp_id_aon, IVC_CARVEOUT_BASE_SS_INDEX,
				carveout_base);
	ret = tegra_hsp_ss_read(&tegra_hsp_id_aon, IVC_CARVEOUT_SIZE_SS_INDEX,
				&carveout_size);
	if (ret)
		goto err;
	tegra_hsp_ss_clear(&tegra_hsp_id_aon, IVC_CARVEOUT_SIZE_SS_INDEX,
				carveout_size);
	dbgprintf_isr("carveout_addr : %x size : %x\r\n",
			(unsigned int)carveout_base,
			(unsigned int)carveout_size);
	if (ivc_carveout_ast_setup(carveout_base, carveout_size))
		return;
	ivc_init_channels_from_ram_ccplex(higher_prio_task_woken);
	ast_inited = true;
	return;
err:
	error_hook("Error reading shared semaphore\n");
}

static void ccplex_ipc_irq(void *dummy, uint32_t data)
{
	BaseType_t higher_prio_task_woken;
	(void)dummy;

	tegra_hsp_sm_vacate(&tegra_hsp_id_aon, MBOX_AON_CCPLEX_IVC_RX);
	data &= ~MBOX_TAG_BIT;

	if (data == SMBOX_IVC_READY_MSG) {
		ivc_ccplex_carveout_configure(&higher_prio_task_woken);
		clk_init_hw();
	} else if (data == SMBOX_IVC_NOTIFY) {
		ivc_channels_ivc_notified_ccplex_ns(&higher_prio_task_woken);
	} else {
		error_hook("spurious MBOX irq");
	}

	portYIELD_FROM_ISR(higher_prio_task_woken);
}

int hsp_ivc_notify_ccplex(struct tegra_ivc_channel *channel, bool is_read)
{
	tegra_hsp_ss_set(&tegra_hsp_id_aon, IVC_NOTIFY_TX_SS, BIT(channel->channel_group));
	tegra_hsp_sm_produce(&tegra_hsp_id_aon, MBOX_AON_CCPLEX_IVC_TX,
				SMBOX_IVC_NOTIFY);
	return 0;
}

int ivc_init_channels_ccplex(void)
{
	int i;
	int ret = 0;

	for (i = 0; i < IVC_NUM_CCPLEX_CHS; i++) {
		if (ivc_ccplex_task_ids[i]->ops->init) {
			ccplex_task_states[i] = ivc_ccplex_task_ids[i]->ops->init(
					ivc_ccplex_task_ids[i]);
			if (!ccplex_task_states[i]) {
				error_hookf("ivc: %s init failed",
						ivc_ccplex_task_ids[i]->name);
				ret = -1;
			}
		}
	}
	tegra_hsp_sm_full_enable(&tegra_hsp_id_aon, MBOX_AON_CCPLEX_IVC_RX, ccplex_ipc_irq,
					NULL);
	return ret;
}
