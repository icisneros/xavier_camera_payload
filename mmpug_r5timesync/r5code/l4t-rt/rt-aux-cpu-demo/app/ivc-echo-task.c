/*
 * Copyright (c) 2015-2016, NVIDIA CORPORATION.  All rights reserved.
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

#include <timeserver.h>

#include <stdio.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>

#include <FreeRTOS.h>
#include <event_groups.h>
#include <semphr.h>
#include <task.h>

#include <err-hook.h>
#include <macros.h>
#include <tegra-ivc.h>
#include <ivc-echo-task.h>
#include <ivc-channels.h>
#include <dbgprintf.h>

#define EVENT_WAIT_BIT			BIT(0)
#define EVENT_IVC_RAM_INITED_BIT	BIT(1)

#define EVENT_WAIT_ALL_BITS	(EVENT_WAIT_BIT | \
				EVENT_IVC_RAM_INITED_BIT)

struct ivc_echo_task_state {
	struct ivc_task_id *id;
	TaskHandle_t handle;
	EventGroupHandle_t wait_event;
	SemaphoreHandle_t ivc_sem;
};

static struct ivc_echo_task_state *ivc_state = NULL;

/*static int ivc_echo_task_write_msg(void *data, int length)
{
	int ret;
	char *tx_msg;
	bool non_contig_available;

	if (!ivc_state) {
		error_hook("ivc_echo_task not init");
		return -1;
	}

	if (length > ivc_state->id->ivc_ch->frame_size) {
		error_hook("length larger than frame buffer size");
		return -1;
	}

	xSemaphoreTake(ivc_state->ivc_sem, portMAX_DELAY);

	ret = tegra_ivc_tx_get_contiguous_write_space(
		ivc_state->id->ivc_ch, &tx_msg, &non_contig_available);
	if (ret < 1) {
		error_hook("tegra_ivc_tx_get_contiguous_write_space() failed");
		goto exit;
	}

	memcpy(tx_msg, data, length);

	ret = tegra_ivc_tx_send_buffers(ivc_state->id->ivc_ch, 1);
	if (ret) {
		error_hook("tegra_ivc_tx_send_buffers() failed");
	}

exit:
	xSemaphoreGive(ivc_state->ivc_sem);
	return ret;
}*/



static void ivc_processByte(char data)
{
	static int parsestate = 0;
	static unsigned int newtickspersecond=0;
	static int 	newphaseoffset = 0;

	if(data == 'f' && parsestate ==0)
	{
		parsestate=1;
	} else if(data == 'r' && parsestate==1)
	{
		parsestate =2;
	} else if(data == 'e' && parsestate ==2)
	{
		parsestate =3;
	}else if(data == 'q' && parsestate == 3)
	{
		parsestate =4;
		newtickspersecond=0;
		newphaseoffset = 0;
	}else if(parsestate>=4 && parsestate < 8)
	{
		int offset = parsestate-4;
		
		newtickspersecond += data <<(8*offset);
		parsestate++;
	} else if(parsestate>=8 && parsestate< 12)
	{
		int offset = parsestate-8;
		newphaseoffset += data <<(8*offset);
		parsestate++;
		//Now we have read all the bytes. Reset and update struct;
		if(parsestate == 12)
		{
			timeserverdata.ticks_persecond = newtickspersecond;
			timeserverdata.phaseoffset_ticks = newphaseoffset;
			timeserverdata.synced=5;
			error_hook("Successfully parsed\n.");
			//In 64 byte increments
			//parsestate = 0;
            parsestate = 0;
            error_hook("Reset from 12th byte\n.");
		}
    } else {
		//error_hook("misalgined timeserver stream");
		parsestate = 0;
	}
}

static void ivc_processByteStream(char *data, int length)
{
	for(int i=0;i<length;++i)
	{
		ivc_processByte(data[i]);
	}
	
}

static void ivc_echo_task_process_ivc_messages(struct ivc_echo_task_state *state)
{
	int ret;
	const char *rx_msg;
	bool non_contig_available;
	int count, i;

	for (;;) {
		xSemaphoreTake(state->ivc_sem, portMAX_DELAY);
		count = tegra_ivc_rx_get_contiguous_read_available(
			state->id->ivc_ch, &rx_msg, &non_contig_available);
		xSemaphoreGive(state->ivc_sem);
        dbgprintf("IVC read count: %d\r\n", count);
		if (count < 0) {
			error_hook("tegra_ivc_rx_get_contiguous_read_available() failed");
			return;
		}
		if (!count) {
			return;
		} else {
			for (i = 0; i < count; i++) {
				ivc_processByteStream((void *)rx_msg, 12);
				//ivc_echo_task_write_msg((void *)rx_msg, 64);
				rx_msg += 64;
			}
		}

        xSemaphoreTake(state->ivc_sem, portMAX_DELAY);
        ret = tegra_ivc_rx_notify_buffers_consumed(state->id->ivc_ch,
            count);
        xSemaphoreGive(state->ivc_sem);
        if (ret) {
            error_hook("tegra_ivc_rx_notify_buffers_consumed() failed");
            return;
        }
	}
}

static portTASK_FUNCTION(ivc_echo_task, pvParameters)
{
	struct ivc_echo_task_state *state = pvParameters;
	EventBits_t ret;

	for (;;) {
		ret = xEventGroupWaitBits(state->wait_event,
					  EVENT_WAIT_ALL_BITS,
					  pdFALSE,
					  pdTRUE,
					  portMAX_DELAY);
		if ((ret & EVENT_WAIT_ALL_BITS) == EVENT_WAIT_ALL_BITS) {
			xEventGroupClearBits(state->wait_event,
					     EVENT_WAIT_BIT);
			ivc_echo_task_process_ivc_messages(state);
		}
	}
}

void *ivc_echo_task_init(struct ivc_task_id *id)
{
	struct ivc_echo_task_state *state;
	int ret;

	state = calloc(1, sizeof(*state));
	if (!state) {
		error_hook("calloc(struct ivc_echo_task_state) failed");
		return NULL;
	}

	state->id = id;

	state->wait_event = xEventGroupCreate();
	if (state->wait_event == NULL) {
		error_hook("xEventGroupCreate() failed");
		free(state);
		return NULL;
	}

	state->ivc_sem = xSemaphoreCreateMutex();
	if (!state->ivc_sem) {
		error_hook("xSemaphoreCreateMutex() failed");
		vEventGroupDelete(state->wait_event);
		free(state);
		return NULL;
	}

	ret = xTaskCreate(ivc_echo_task, "ivcecho", 512, state, 1,
		&state->handle);
	if (ret != pdPASS) {
		error_hook("xTaskCreate() failed");
		vEventGroupDelete(state->wait_event);
		vSemaphoreDelete(state->ivc_sem);
		free(state);
		return NULL;
	}

	ivc_state = state;

	return state;
}

int ivc_echo_task_ivc_notified(void *statev, BaseType_t *higher_prio_task_woken)
{
	struct ivc_echo_task_state *state = statev;
	BaseType_t ret;

	ret = xEventGroupSetBitsFromISR(state->wait_event,
					EVENT_WAIT_BIT,
					higher_prio_task_woken);
	if (ret != pdPASS) {
		error_hook("xEventGroupSetBitsFromISR() failed");
		return -1;
	}

	return 0;
}

int ivc_echo_task_ivc_inited(void *statev, BaseType_t *higher_prio_task_woken)
{
	struct ivc_echo_task_state *state = statev;
	BaseType_t ret;

	ret = xEventGroupSetBitsFromISR(state->wait_event,
					EVENT_IVC_RAM_INITED_BIT,
					higher_prio_task_woken);
	if (ret != pdPASS) {
		error_hook("xEventGroupSetBitsFromISR() failed");
		return -1;
	}

	return 0;
}
