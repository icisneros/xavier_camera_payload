/*
 * Copyright (c) 2015, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _TEGRA_AODMIC_PRIV_H_
#define _TEGRA_AODMIC_PRIV_H_

#include <stdint.h>

#include <FreeRTOS.h>
#include <semphr.h>
#include <queue.h>

#include <clk-tegra.h>
#include <tegra-gpcdma.h>
#include <tegra-aodmic.h>

/* Internal hardcodings of AODMIC */
#define TEGRA_AODMIC_WORD_SIZE        4   /* 4 bytes  */
#define TEGRA_AODMIC_FIFO_SIZE        32  /* 32 words */

/* AODMIC interrupt threshold */
#define TEGRA_AODMIC_FIFO_THRESHOLD   (TEGRA_AODMIC_FIFO_SIZE / 2)

struct tegra_aodmic_id {
	const char *devname;
	uint32_t base_addr;
	const struct tegra_clk *clk;
	const struct tegra_rst *rst;
	uint32_t irq;
};

struct tegra_aodmic_ctlr {
	/*
	 * The driver relies on this being the first field, since it casts
	 * pointers between structs tegra_aodmic_id and tegra_aodmic_ctlr
	 */
	struct tegra_aodmic_id id;

	/* PCM configuration */
	uint32_t sample_rate;  /* In Hz */
	uint32_t num_channels; /* Channel count */
	uint32_t samp_bytes;   /* Bytes per sample */

	/* GPCDMA configuration */
	struct tegra_gpcdma_id    *gpcdma_id;   /* DMA instance ID */
	int32_t                   dma_chan_num; /* DMA channel no. */
	struct tegra_gpcdma_xfer  dma_xfer;     /* DMA transfer parameters */

	/* Driver data buffer */
	uint8_t  *data;         /* Data pointer (of num_periods length) */
	uint32_t period_bytes;  /* Size of each period in data buffer */
	uint32_t num_periods;   /* No. of periods */

	/* Buffer control */
	SemaphoreHandle_t xSemaphore_free_buf; /* Count of free periods */
	QueueHandle_t xQueue_filled_buf;   /* Queue of filled periods */
	TickType_t        max_read_timeout;    /* Timeout at queue receive */
	uint32_t free_pos;      /* Next available free period (for writing) */
	uint8_t  *read_ptr;     /* Location of period to read */
	uint32_t bytes_to_read; /* Bytes available to read */

	/* Control information */
	uint32_t init_done;
	uint32_t dma_running;
};

#endif
