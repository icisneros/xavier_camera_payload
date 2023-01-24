/* Copyright (c) 2015-2017, NVIDIA CORPORATION.  All rights reserved.
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

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include <address_map_new.h>
#include <err-hook.h>
#include <hsp-tegra.h>
#include <hsp-tegra-hw.h>
#include <macros.h>
#include <mbox-aon.h>
#include <reg-access.h>

#include <bpmp-ipc.h>
#include <bpmp-ipc-protocol.h>
#include <spe-pm.h>

#include <dbgprintf.h>

#define MAX_MESSAGES	31

struct bpmp_ipc_msg {
	uint32_t data;
	bool dir_bpmp_to_spe;
};

static QueueHandle_t bpmp_ipc_queue;

static void bpmp_ipc_irq(void *dummy, uint32_t data)
{
	struct bpmp_ipc_msg msg;
	BaseType_t higher_prio_task_woken;
	(void)dummy;

	tegra_hsp_sm_vacate(&tegra_hsp_id_aon, MBOX_AON_BPMP);
	data &= ~MBOX_TAG_BIT;

	msg.data = data;
	msg.dir_bpmp_to_spe = true;
	if (xQueueSendToBackFromISR(bpmp_ipc_queue, &msg, &higher_prio_task_woken) != pdPASS) {
		error_hook("xQueueSendFromISR() failed");
	}

	portYIELD_FROM_ISR(higher_prio_task_woken);
}

void bpmp_ipc_send_now_from_isr(uint32_t data)
{
	tegra_hsp_sm_produce(&tegra_hsp_id_bpmp, MBOX_BPMP_BPMP, data);
}

void bpmp_ipc_send_now(uint32_t data)
{
	vPortEnterCritical();
	tegra_hsp_sm_produce(&tegra_hsp_id_bpmp, MBOX_BPMP_BPMP, data);
	vPortExitCritical();
}

int bpmp_ipc_send_from_isr(uint32_t data, BaseType_t *higher_prio_task_woken)
{
	struct bpmp_ipc_msg msg;
	int ret = 0;

	msg.data = data;
	msg.dir_bpmp_to_spe = false;
	if (xQueueSendToBackFromISR(bpmp_ipc_queue, &msg, higher_prio_task_woken) != pdPASS) {
		error_hook("xQueueSendToBack() failed");
		ret = -1;
	}

	return ret;
}

int bpmp_ipc_send(uint32_t data)
{
	struct bpmp_ipc_msg msg;
	int ret = 0;

	msg.data = data;
	msg.dir_bpmp_to_spe = false;
	if (xQueueSendToBack(bpmp_ipc_queue, &msg, 0) != pdPASS) {
		error_hook("xQueueSendToBack() failed");
		ret = -1;
	}

	return ret;
}

static void bpmp_ipc_task_process_ipc_queue(struct bpmp_ipc_msg *msg)
{
	if (msg->dir_bpmp_to_spe) {
		switch (msg->data) {
		case BPMP_IPC_MSG_BPMP_PING:
			dbgprintf("ping successful from BPMP to SPE\r\n");
			bpmp_ipc_send_now(BPMP_IPC_MSG_SPE_PING);
			break;
		case BPMP_IPC_MSG_SPE_PING_ACK:
			dbgprintf("ping successful from SPE to BPMP\r\n");
			break;
		case BPMP_IPC_MSG_DRAM_WAKE_REQ_ACK:
			dbgprintf("DRAM awake msg from BPMP\r\n");
			/* FIXME: Is this the right call for DRAM awake */
			spe_exit_sc7();
			break;
		case BPMP_IPC_MSG_SC7_ENTRY:
			dbgprintf("BPMP informing SPE of SC7 entry\r\n");
			spe_prepare_enter_sc7();
			break;
		case BPMP_IPC_MSG_SC7_EXIT:
			dbgprintf("BPMP informing SPE of SC7 exit\r\n");
			spe_exit_sc7();
			break;
		default:
			error_hookf("Invalid IPC message from BPMP: %u\r\n", (unsigned int)msg->data);
			break;
		}
	} else {
		if (msg->data <= 0 || msg->data > BPMP_IPC_MSG_ID_MAX) {
			error_hookf("SPE -> BPMP: Invalid IPC message: %u\r\n", (unsigned int)msg->data);
			return;
		}
		dbgprintf("SPE -> BPMP: %u\r\n", (unsigned int)msg->data);
		bpmp_ipc_send_now(msg->data);
	}
}

static portTASK_FUNCTION(bpmp_ipc_task, pvParameters)
{
	(void)pvParameters;
	struct bpmp_ipc_msg msg;
	BaseType_t qret;

	for (;;) {
		qret = xQueueReceive(bpmp_ipc_queue, &msg, portMAX_DELAY);
		if (qret != pdTRUE) {
			error_hook("xQueueReceive() failed");
			continue;
		}
		bpmp_ipc_task_process_ipc_queue(&msg);
	}
}

int bpmp_ipc_init(void)
{
	BaseType_t xStatus;

	bpmp_ipc_queue = xQueueCreate(MAX_MESSAGES, sizeof(struct bpmp_ipc_msg));
	if (!bpmp_ipc_queue) {
		error_hook("BPMP IPC xQueueCreate() failed");
		return -1;
	}
	xStatus = xTaskCreate(bpmp_ipc_task, "bpmp_ipc", 512, NULL, 2, NULL);
	if (!xStatus) {
		error_hook("bpmp_ipc_task creation failed");
		vQueueDelete(bpmp_ipc_queue);
		bpmp_ipc_queue = NULL;
		return -1;
	}
	tegra_hsp_sm_full_enable(&tegra_hsp_id_aon, MBOX_AON_BPMP, bpmp_ipc_irq,
			NULL);

	return 0;
}
