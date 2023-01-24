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

/*
 * Creates one task that repeatedly reads AODMIC data using GPCDMA;
 * prints zero crossing count and avg volume (mean square) periodically;
 * triggers system wake if computed volume crosses a threshold
 */

/*
 * NOTE: SPE should not enter its low power states while running this app
 */

#include <stdio.h>
#include <stdlib.h>
#include <FreeRTOS.h>
#include <task.h>
#include <tegra-gpcdma.h>
#include <tegra-gpcdma-hw.h>
#include <tegra-aodmic.h>
#include <tegra-aodmic-hw.h>
#include <wake-tegra.h>
#include <wake-tegra-hw.h>
#include <err-hook.h>

#include "aodmic-app.h"

#define TEST_AODMIC         tegra_aodmic_id_aon

/* GPCDMA channel - need to ensure that no other */
/* application uses the same DMA channel         */
#define AODMIC_DMA_CHANNEL  7

/* Volume threshold for triggering system wake */
#define SPE_CCPLEX_WAKE_THRESHOLD 1000000

static void wake_trigger_ccplex_wake(void)
{
	tegra_wake_trigger_wake_event(&tegra_wake_id_wake);
}

static portTASK_FUNCTION(aodmic_test_task, pvParameters)
{
	int status = 0, aodmic_opened = 0;
	int i, buf_size, period_ms;
	(void)pvParameters; /* unused */
	struct tegra_aodmic_config config;
	char err_msg[100];
	TickType_t start_time, end_time;
	int j, zero_cross_l = 0, zero_cross_r = 0;
	unsigned int vol_l = 0, vol_r = 0, temp_l, temp_r;
	int16_t *buf = NULL;
	int16_t prev_samp_l = 0, prev_samp_r = 0;
	int16_t cur_samp_l, cur_samp_r;

	/* AODMIC configuration (editable) */
	config.sample_rate    = TEGRA_AODMIC_RATE_16KHZ;
	config.channel_config = TEGRA_AODMIC_CHANNEL_STEREO;
	config.sample_width   = TEGRA_AODMIC_BITS_PER_SAMPLE_16;
	config.period_size    = 256;
	config.num_periods    = 2;

	/* GPCDMA configuration */
	config.gpcdma_id      = &tegra_gpcdma_id_aon;
	config.dma_chan_num   = AODMIC_DMA_CHANNEL;

	/* Below is dependent on channels and bit width */
	buf_size = config.period_size * 2 * sizeof(int16_t);
	/* Below is dependent on sample rate */
	period_ms = config.period_size / 16;

	buf = malloc(buf_size);
	if (buf == NULL) {
		sprintf(err_msg, "aodmic_test: Unable to allocate data buffer");
		goto err_exit;
	}

	printf("aodmic_test: Initializing GPCDMA channel %d...\n\r",
	       (int)config.dma_chan_num);
	status = tegra_gpcdma_channel_init(config.gpcdma_id,
						config.dma_chan_num);
	if (status != 0) {
		sprintf(err_msg, "aodmic_test: Error in tegra_gpcdma_channel_init: %x", status);
		goto err_exit;
	}

	printf("aodmic_test: Opening AODMIC...\n\r");
	status = tegra_aodmic_open(&TEST_AODMIC, &config);
	if (status) {
		sprintf(err_msg, "aodmic_test: Couldn't initialize AODMIC: %x", status);
		goto err_exit;
	}
	aodmic_opened = 1;

	printf("aodmic_test: Starting AODMIC read...\n\r");
	start_time = xTaskGetTickCount();
	for (i = 1; ; i++) {
		status = tegra_aodmic_read(&TEST_AODMIC, buf, buf_size);
		if (status) {
			sprintf(err_msg, "aodmic_test: Couldn't read from AODMIC: %x", status);
			vTaskDelay(period_ms);
			continue;
		}

		temp_l = 0;
		temp_r = 0;
		for (j = 0; j < (buf_size / sizeof(int16_t)); j += 2) {
			cur_samp_l = buf[j];
			cur_samp_r = buf[j+1];

			/* Count zero crossings on both channels */
			if (((prev_samp_l < 0) && (cur_samp_l >= 0)) ||
			    ((prev_samp_l > 0) && (cur_samp_l <= 0))) {
				zero_cross_l++;
			}
			if (((prev_samp_r < 0) && (cur_samp_r >= 0)) ||
			    ((prev_samp_r > 0) && (cur_samp_r <= 0))) {
				zero_cross_r++;
			}
			prev_samp_l = cur_samp_l;
			prev_samp_r = cur_samp_r;

			/* Accumulate capture volume on both channels */
			temp_l += cur_samp_l * cur_samp_l;
			temp_r += cur_samp_r * cur_samp_r;

		}
		vol_l += (temp_l / config.period_size);
		vol_r += (temp_r / config.period_size);

		if ((i % 30) == 0) {
			/* Print tick count and zero crossing count - */
			/*  useful to check correct AODMIC rate       */
			end_time = xTaskGetTickCount();
			printf("aodmic_test: Ticks: %u; Zero Crossings: L(%d) R(%d)\n\r",
			       (unsigned int)
			       ((end_time - start_time) * portTICK_PERIOD_MS),
			       zero_cross_l, zero_cross_r);
			zero_cross_l = 0;
			zero_cross_r = 0;
			start_time = xTaskGetTickCount();

			/* CCPLEX wake - on volume trigger */
			vol_l /= 30;
			vol_r /= 30;
			printf("aodmic_test: Computed volume: L(%u) R(%u)\n\r",
			       vol_l, vol_r);
			if ((vol_l >= SPE_CCPLEX_WAKE_THRESHOLD) ||
			    (vol_r >= SPE_CCPLEX_WAKE_THRESHOLD)) {
				printf("aodmic_test: !!! TRIGGERING CCPLEX WAKE !!!\n\r");
				wake_trigger_ccplex_wake();
			}
			vol_l = 0;
			vol_r = 0;
		}
	}

err_exit:
	error_hook(err_msg);

	if (aodmic_opened)
		tegra_aodmic_close(&TEST_AODMIC);
	if (buf != NULL)
		free(buf);

	vTaskDelete(NULL);
}

void aodmic_app_init(void)
{
	printf("aodmic_test: Launching task...\n\r");
	xTaskCreate(aodmic_test_task, "aodmictest", 1024, NULL,
			tskIDLE_PRIORITY, (TaskHandle_t *)NULL);
}
