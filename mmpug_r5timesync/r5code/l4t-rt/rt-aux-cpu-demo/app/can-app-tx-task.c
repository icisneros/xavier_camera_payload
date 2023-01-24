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
#include <semphr.h>
#include <task.h>
#include <tegra-can.h>
#include <tegra-mttcan.h>
#include <can-tegra-hw.h>
#include <tke-tegra.h>

#include "can-app-tx-task.h"
#include "can-app-common.h"

#define CAN_TEST_DELAY			10000
#define PRINTF_RATE_LIMIT		25

static void can_process_txevent(uint32_t elm0, uint32_t elm1)
{
	uint8_t id_size = (elm0 >> 30) & 0x1;
	uint32_t id = id_size ? elm0 & 0x1fffffff : (elm0 >> 18) & 0x7ff;
	uint8_t event_type = (elm1 >> 22) & 0x3;
	uint8_t can_frame_format = (elm1 >> 21) & 0x1;
	uint8_t can_brs = (elm1 >> 20) & 0x1;

	printf("Transmit event element information:\r\n");
	printf("Message ID: 0x%x, Event Type: %s, CAN Frame: %s, %s, %s\r\n",
		(unsigned int)id, (event_type == 1) ? "Tx event" :
		(event_type == 2) ? "Tx in spite of cancellation" : "Reserved",
		(can_frame_format == 1) ? "CAN FD" : "Standard Frame",
		(id_size == 1) ? "29bit ID" : "11bit ID",
		(can_brs == 1) ? "BRS" : "No BRS");
}

/*
 * CAN creates transmit event elements on each mttcan_transfer, this task reads
 * out the event elements from the internal CAN message RAM.
 */
static portTASK_FUNCTION(can_tx_event, pvParameters)
{
	struct ttcan_controller *ttcan = (struct ttcan_controller *)pvParameters;
	struct mttcanfd_frame cf;
	int i = 0;

	while (1) {
		/* Block on CAN ISR to get data */
		if (!tegra_can_receive(ttcan, &cf, portMAX_DELAY))
			continue;

		switch (cf.cmdid) {
		case MTTCAN_MSG_TX_COMPL:
			printf("Transmission complete event\r\n");
			break;
		case MTTCAN_MSG_TXEVT:
			/*
			 * Each transmit event element is two words long, we
			 * will selectively parse fields of both the elements
			 */
			can_process_txevent(cf.payload.data[0],
					    cf.payload.data[1]);
			break;
		case MTTCAN_MSG_BERR_CHG:
			if (i++ < PRINTF_RATE_LIMIT)
				printf("Bus Error\r\n");
			break;
		case MTTCAN_MSG_STAT_CHG:
			printf("Bus state change\r\n");
			break;
		default:
			printf("Commnad id: 0x%x not processed\r\n", cf.cmdid);
			break;
		}
	}
}

static void can_test_transmit(struct mttcanfd_frame *fd,
			      struct ttcan_controller *ttcan, bool is_xtd,
			      bool is_can_fd, bool is_brs)
{
	if (is_xtd)
		fd->payload.frame.can_id |= (CAN_FMT);
	else
		fd->payload.frame.can_id &= ~(CAN_FMT);

	if (is_can_fd && is_brs)
		fd->payload.frame.flags |= (CAN_FD_FLAG) | (CAN_BRS_FLAG);
	else if (is_can_fd)
		fd->payload.frame.flags |= (CAN_FD_FLAG);
	else
		fd->payload.frame.flags &= ~((CAN_FD_FLAG) | (CAN_BRS_FLAG));

	fd->payload.frame.tstamp = tegra_tke_get_usec();
	mttcan_transfer(ttcan, fd);
	printf("Transmited message from CAN %lu\r\n", ttcan->id);
	vTaskDelay(CAN_TEST_DELAY);
}

static portTASK_FUNCTION(can_xmit_test, pvParameters)
{
	struct ttcan_controller *ttcan = (struct ttcan_controller *)pvParameters;
	struct mttcanfd_frame fd;

	memset(&fd, 0, sizeof(fd));
	/* cmdid information can be found in tegra-can.h */
	fd.cmdid = MTTCAN_MSG_TX;
	/*
	 * can_id and data are just random numbers, information regarding can_id
	 * and flags bit fields can be found in tegra-can.h.
	 */
	fd.payload.frame.can_id = 0xa5;
	fd.payload.frame.d_len = 2;
	fd.payload.frame.flags = 0x0;
	fd.payload.frame.data[0] = 0xaa;
	fd.payload.frame.data[1] = 0x55;

	while (1) {
		/* Normal CAN frame with 11 bit identifier */
		can_test_transmit(&fd, ttcan, false, false, false);
		/* Normal CAN frame with 29 bit identifier */
		can_test_transmit(&fd, ttcan, true, false, false);
		/* CAN FD frame with 11 bit identifier */
		can_test_transmit(&fd, ttcan, false, true, false);
		/* CAN FD frame with 29 bit identifier */
		can_test_transmit(&fd, ttcan, true, true, false);
		/* CAN FD BRS frame with 11 bit identifier */
		can_test_transmit(&fd, ttcan, false, true, true);
		/* CAN FD BRS frame with 29 bit identifier */
		can_test_transmit(&fd, ttcan, true, true, true);
	}
}

BaseType_t can_app_tx_task_init(struct ttcan_controller *ttcan)
{
	BaseType_t ret;
	TaskHandle_t can_xmit_handle, can_tx_eve_handle;

	if (!ttcan)
		return pdFAIL;

	/* Setup CAN controller */
	ret = can_init(ttcan);
	if (ret != pdPASS) {
		printf("Unable to initialize CAN%lu controller\r\n", ttcan->id);
		return ret;
	}

	/* CAN TX task */
	ret = xTaskCreate(can_xmit_test, 0, 512, ttcan, tskIDLE_PRIORITY,
			  &can_xmit_handle);
	if (ret != pdPASS) {
		printf("CAN%lu: xTaskCreate for transmit failed\r\n", ttcan->id);
		goto can_deinit_err;
	}

	ret = xTaskCreate(can_tx_event, 0, 512, ttcan, tskIDLE_PRIORITY,
			  &can_tx_eve_handle);
	if (ret != pdPASS) {
		printf("CAN%lu: xTaskCreate for receive failed\r\n", ttcan->id);
		goto xmit_test_delete;
	}

	return ret;

xmit_test_delete:
	vTaskDelete(can_xmit_handle);
can_deinit_err:
	can_deinit(ttcan);

	return ret;
}
