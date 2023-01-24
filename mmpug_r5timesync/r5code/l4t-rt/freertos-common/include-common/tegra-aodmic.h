/*
* Copyright (c) 2015 NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _TEGRA_AODMIC_H_
#define _TEGRA_AODMIC_H_

#include <stdint.h>

#include <tegra-gpcdma.h>

/* Error codes */
#define TEGRA_AODMIC_ERR_BASE                   (0x80001000)
#define TEGRA_AODMIC_ERR_CONFIGURATION_NA       (TEGRA_AODMIC_ERR_BASE + 0x01)
#define TEGRA_AODMIC_ERR_INVALID_SAMPLE_RATE    (TEGRA_AODMIC_ERR_BASE + 0x02)
#define TEGRA_AODMIC_ERR_CLK_RATE_ERROR         (TEGRA_AODMIC_ERR_BASE + 0x03)
#define TEGRA_AODMIC_ERR_INVALID_CHANNELS       (TEGRA_AODMIC_ERR_BASE + 0x04)
#define TEGRA_AODMIC_ERR_INVALID_SAMPLE_WIDTH   (TEGRA_AODMIC_ERR_BASE + 0x05)
#define TEGRA_AODMIC_ERR_INVALID_PERIOD         (TEGRA_AODMIC_ERR_BASE + 0x06)
#define TEGRA_AODMIC_ERR_MEM_ALLOC_FAILURE      (TEGRA_AODMIC_ERR_BASE + 0x07)
#define TEGRA_AODMIC_ERR_SEM_CREATE_FAILURE     (TEGRA_AODMIC_ERR_BASE + 0x08)
#define TEGRA_AODMIC_ERR_NOT_INITIALIZED        (TEGRA_AODMIC_ERR_BASE + 0x09)
#define TEGRA_AODMIC_ERR_BUF_INVALID            (TEGRA_AODMIC_ERR_BASE + 0x0A)
#define TEGRA_AODMIC_ERR_DMA_XFER_ERROR         (TEGRA_AODMIC_ERR_BASE + 0x0B)
#define TEGRA_AODMIC_ERR_READ_TIMEOUT           (TEGRA_AODMIC_ERR_BASE + 0x0C)

/* Supported sample rates */
enum tegra_aodmic_sample_rate {
	TEGRA_AODMIC_RATE_8KHZ,
	TEGRA_AODMIC_RATE_16KHZ,
	TEGRA_AODMIC_RATE_44KHZ,
	TEGRA_AODMIC_RATE_48KHZ,

	TEGRA_AODMIC_NUM_RATES
};

/* Supported channel configurations */
enum tegra_aodmic_channel_config {
	TEGRA_AODMIC_CHANNEL_MONO_LEFT,
	TEGRA_AODMIC_CHANNEL_MONO_RIGHT,
	TEGRA_AODMIC_CHANNEL_STEREO,

	TEGRA_AODMIC_NUM_CHANNEL_CONFIGS
};

/*
 * Supported sample widths
 *  AODMIC provides samples at 24 bit sample width,
 *  which will be scaled as per below configuration
 */
enum tegra_aodmic_sample_width {
	TEGRA_AODMIC_BITS_PER_SAMPLE_16,
	TEGRA_AODMIC_BITS_PER_SAMPLE_32,

	TEGRA_AODMIC_NUM_SAMPLE_WIDTHS
};

/*
 * Config structure for AODMIC
 * PCM configuration:
 *  sample_rate:        As per tegra_aodmic_sample_rate
 *  channel_config:     As per tegra_aodmic_channel_config
 *  sample_width:       As per tegra_aodmic_sample_width
 * DMA configuration:
 *  period_size:        DMA interrupt interval (in sample frames),
 *                      must be multiple of 16
 *  num_periods:        Must be >= 2; total buffer with driver =
 *                      (period_size * num_periods)
 * DMA instance:
 *  gpcdma_id           GPCDMA instance handle
 *  dma_chan_num:       GPCDMA channel no. to use (0 - 7)
 *  @@ NOTE THAT GPCDMA CONTROLLER AND CHANNEL SHOULD BE
 *  @@ INITIALIZED PRIOR TO CALLING tegra_aodmic_open()
 */
struct tegra_aodmic_config {
	/* PCM configuration */
	enum tegra_aodmic_sample_rate    sample_rate;
	enum tegra_aodmic_channel_config channel_config;
	enum tegra_aodmic_sample_width   sample_width;

	/* DMA configuration */
	uint32_t  period_size;
	uint32_t  num_periods;

	/* DMA instance */
	struct tegra_gpcdma_id  *gpcdma_id;
	int32_t                 dma_chan_num;
};

struct tegra_aodmic_id;

/*
 * Open and initialize AODMIC instance
 *
 * Parameters:
 * id:          AODMIC controller instance
 * config:      Configuration structure
 *
 * Returns:
 * 0 on success
 * Non-zero error code on error.
 */
int tegra_aodmic_open(struct tegra_aodmic_id *id,
                      struct tegra_aodmic_config *config);

/*
 * Read captured samples
 *
 * Parameters:
 * id:          AODMIC controller instance
 * data:        Data buffer pointer
 * count:       No. of bytes to capture
 *
 * Returns:
 * 0 on success
 * Non-zero error code on error.
 */
int tegra_aodmic_read(struct tegra_aodmic_id *id, void *data, uint32_t count);

/*
 * Close AODMIC instance
 *
 * Parameters:
 * id:          AODMIC controller instance
 *
 * Returns:
 * None
 */
void tegra_aodmic_close(struct tegra_aodmic_id *id);

#endif
