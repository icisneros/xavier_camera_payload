/*
 * Copyright (c) 2018 NVIDIA CORPORATION. All rights reserved.
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

#include <hsp-tegra-hw.h>

#include <mbox-aon.h>
#include <combined-uart.h>
#include <combined-uart-priv.h>

struct ext_client_id ext_client_ids[] = {
	[COMBINED_UART_CCPLEX] = {
		.tag = 0xe1,
		.name = "ccplex",
		.hsp_id = &tegra_hsp_id_top0,
		.recv_mbox = MBOX_AON_CCPLEX_UART,
		.send_mbox = 0,
		.enabled = true,
	},
	[COMBINED_UART_BPMP] = {
		.tag = 0xe2,
		.name = "bpmp",
		.hsp_id = &tegra_hsp_id_bpmp,
		.recv_mbox = MBOX_AON_BPMP_UART,
		.send_mbox = MBOX_BPMP_UART,
		.enabled = true,
	},
	[COMBINED_UART_SCE]  = {
		.tag = 0xe3,
		.name = "sce",
		.hsp_id = &tegra_hsp_id_sce,
		.recv_mbox = MBOX_AON_SCE_UART,
		.send_mbox = MBOX_SCE_UART,
		.enabled = true,
	},
	[COMBINED_UART_TZ]  = {
		.tag = 0xe4,
		.name = "tz",
		.recv_mbox = MBOX_AON_TZ_UART,
		.enabled = true,
	},
	[COMBINED_UART_RCE]  = {
		.tag = 0xe5,
		.name = "rce",
		.recv_mbox = MBOX_AON_RCE_UART,
		.enabled = true,
	},
};
