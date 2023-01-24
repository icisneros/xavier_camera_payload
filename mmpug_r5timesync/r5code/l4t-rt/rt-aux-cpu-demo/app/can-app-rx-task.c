/*
 * Copyright (c) 2020, NVIDIA CORPORATION.  All rights reserved.
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

#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <FreeRTOS.h>
#include <task.h>
#include <tegra-can.h>
#include <tegra-mttcan.h>
#include <can-tegra-hw.h>

#include "can-app-rx-task.h"
#include "can-app-common.h"

static void can_rx_test_print(struct ttcan_controller *ttcan,
			      struct mttcanfd_frame *cf)
{
	int msgid, dlen, i;
	struct canfd_frame frame;

	/* Process debug print only if packet is a CAN RX message */
	if (cf->cmdid != MTTCAN_MSG_RX) {
		printf("Message received with command id (0x%02x) different"
			" than MTTCAN_MSG_RX at CAN %lu\r\n", cf->cmdid,
			ttcan->id);
		return;
	}

	/* Update canfd_frame based on data from CAN message RAM */
	ttcan_read_rx_msg_ram(ttcan, (uint32_t)&cf->payload.data[0], &frame);
	msgid = frame.can_id;
	dlen = frame.d_len;
	printf("Message received at CAN %lu\r\n", ttcan->id);
	printf("Message ID: 0x%02x, Message data length: %d\r\n", msgid, dlen);
	printf("Message Data:");
	for (i = 0; i < frame.d_len; i++) {
		if (!(i & 7))
			printf("\r\n");
		printf("0x%02x ", frame.data[i]);
	}
	printf("\r\n");
}

/* Waits on CAN RX queue for messages */
static portTASK_FUNCTION(can_rx_test, pvParameters)
{
	struct ttcan_controller *ttcan = (struct ttcan_controller *)pvParameters;
	struct mttcanfd_frame cf;

	while (1) {
		/* Block on CAN ISR to get data */
		if (!tegra_can_receive(ttcan, &cf, portMAX_DELAY))
			continue;
		can_rx_test_print(ttcan, &cf);
	}
}

BaseType_t can_app_rx_task_init(struct ttcan_controller *ttcan)
{
	BaseType_t ret;
	TaskHandle_t can_rx_handle;

	if (!ttcan)
		return pdFAIL;

	/* Setup CAN controller */
	ret = can_init(ttcan);
	if (ret != pdPASS) {
		printf("Unable to initialize CAN%lu controllers\r\n", ttcan->id);
		return ret;
	}

	/* CAN RX task */
	ret = xTaskCreate(can_rx_test, 0, 512, ttcan, tskIDLE_PRIORITY,
			  &can_rx_handle);
	if (ret != pdPASS) {
		printf("CAN%lu: xTaskCreate for receive failed\r\n", ttcan->id);
		goto can_deinit_err;
	}

	return ret;

can_deinit_err:
	can_deinit(ttcan);

	return ret;
}
