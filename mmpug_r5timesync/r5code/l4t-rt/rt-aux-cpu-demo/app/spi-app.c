/*
 * Copyright (c) 2019, NVIDIA CORPORATION.  All rights reserved.
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
#include <FreeRTOS.h>
#include <task.h>
#include <macros.h>
#include <argpcdma_ao.h>
#include <spi-tegra-hw.h>
#include <tegra-gpcdma.h>
#include <tegra-gpcdma-hw.h>

#include "spi-app.h"

#define SPI_TEST_CONTROLLER	tegra_spi_id_spi2
#define SPI_TEST_CLOCK_RATE	12000000
#define SPI_TEST_DMA_TX_CHANNEL	2
#define SPI_TEST_DMA_RX_CHANNEL	3

#define SPI_TEST_RETRIES	5
#define SPI_TEST_DELAY		5000

static portTASK_FUNCTION(spi_test_task, pvParameters)
{
	const uint8_t data_to_send[] = {0xab, 0xcd};
	uint8_t data_to_read[] = {0x0, 0x0};
	int ret, count;
	struct tegra_spi_xfer xfer = {
		.flags = BIT(TEGRA_SPI_XFER_FIRST_MSG) |
			 BIT(TEGRA_SPI_XFER_LAST_MSG),
		.tx_buf = data_to_send,
		.rx_buf = data_to_read,
		.len = ARRAY_SIZE(data_to_read),
		.chip_select = 0,
		.tx_nbits = TEGRA_SPI_NBITS_SINGLE,
		.rx_nbits = TEGRA_SPI_NBITS_SINGLE,
		.bits_per_word = 8,
		.mode = TEGRA_SPI_MODE_0 | TEGRA_SPI_LSBYTE_FIRST,
	};

	(void)pvParameters; /* unused */
	for (count = 0; count < SPI_TEST_RETRIES; count++) {
		ret = tegra_spi_transfer(&SPI_TEST_CONTROLLER, &xfer);
		if (ret)
			printf("SPI TX/RX failed\r\n");
		else {
			if (!memcmp(data_to_read, data_to_send,
				    ARRAY_SIZE(data_to_read)))
				printf("SPI test successful\r\n");
			else
				printf("Received incorrect data\r\n");
		}
		vTaskDelay(SPI_TEST_DELAY);
	}
	vTaskDelete(NULL);
}

void spi_app_init(void)
{
	int ret;
	struct tegra_spi_client_setup spi_test_device[] = {
		{
			.chip_select = 0,
			.set_rx_tap_delay = false,
			.spi_max_clk_rate = SPI_TEST_CLOCK_RATE,
			.spi_no_dma = false,
		}
	};
	struct tegra_spi_master_init master_test_conf[] = {
		{
			.dma_id = &tegra_gpcdma_id_aon,
			.dma_channel.tx = SPI_TEST_DMA_TX_CHANNEL,
			.dma_channel.rx = SPI_TEST_DMA_RX_CHANNEL,
			.spi_max_clk_rate = SPI_TEST_CLOCK_RATE,
			.dma_slave_req = GPCDMA_AO_CHANNEL_CH0_CSR_0_REQ_SEL_SPI,
		}
	};

	ret = tegra_spi_init(&SPI_TEST_CONTROLLER, master_test_conf);
	if (ret) {
		printf("spi_test: master init failed\r\n");
		return;
	}

	ret = tegra_spi_setup(&SPI_TEST_CONTROLLER, spi_test_device);
	if (ret) {
		printf("spi_test: couldn't setup SPI device\r\n");
		return;
	}

	xTaskCreate(spi_test_task, "spitest", 512, NULL, tskIDLE_PRIORITY,
		    (TaskHandle_t *)NULL);
}
