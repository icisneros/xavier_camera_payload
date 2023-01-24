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

#ifndef _GTE_TEGRA_H_
#define _GTE_TEGRA_H_

#include <stdbool.h>
#include <stdint.h>

#include <dma.h>

struct tegra_gte_id;

/*
 * Structure for GTE timestamp data
 *
 * Field:
 * tsc:       Value of TSC for the captured event
 * slice:     Indicate the slice for the captured event
 * bit_index: Bit index in the slice of bit that flipped between the current
 *            and previous value
 * bit_dir:   Direction of the change for the identified bit
 *            0: rising edge
 *            1: falling edge
 * invalid:   Indicates that the other field do not carry any valid information
 */
struct tegra_gte_ts {
	uint64_t tsc;
	uint8_t slice;
	uint8_t bit_index;
	bool bit_dir;
	bool invalid;
};

/*
 * This structure represents the in-memory layout of a message extracted from
 * the GTE FIFO
 */
struct tegra_gte_fifo_msg {
	uint32_t ctrl;
	uint32_t tsc_hi;
	uint32_t tsc_lo;
	uint32_t src;
	uint32_t ccv;
	uint32_t pcv;
	uint32_t encv;
	uint32_t cmd;
};

/*
 * IRQ callback function
 *
 * Parameters:
 * data:      data context for irq
 * ts_data:   FIFO data will be stored here
  */
typedef void tegra_gte_callback(void *data, struct tegra_gte_ts *ts_data);

/*
 * DMA transfer function
 *
 * Parameters:
 * *src:           The source address to read from
 * *dst:           The target address to write to
 * len:            The number of bytes to be transferred.
 * io_burst_size:  The number of words to transfer in each burst.
 * src_addr_wrap:  The number of word transfers to make before wrapping
 *                 the src address. 0 represents no wrap.
 *                 Legal values for io addr wrap are 2^n with n from
 *                 0 through 6.
 *                 Legal values for mc addr wrap are 2^n with n from
 *                 5 through 11.
 * Returns:
 * An enum dma_status value. See dma.h for details.
 */
typedef enum dma_status tegra_gte_dma_xfer(void *src, void *dst, uint32_t len,
					   uint32_t io_burst_size,
					   uint32_t src_addr_wrap);

/*
 * Tegra GTE driver
 *
 * To use this driver without DMA
 * 1. Call tegra_gte_slice_set_enable_mask to enable which events to be monitored
 * 2. Call tegra_gte_setup with "dma_en = 0"
 *
 * To use this driver with DMA
 * 1. Initialize DMA
 * 2. Implement a dma transfer function as asynchronous transfer and call
 *    tegra_gte_dma_complete_handler once the transfer is done
 * 3. Call tegra_gte_slice_set_enable_mask to enable which events to be monitored
 * 4. Call tegra_gte_setup with a dma transfer fucntion pointer
 *    and set "dma_en = 1"
 */

/*
 * DMA conplete handler
 *
 * This function should be called once dma transfer is done
 * Parameters:
 * callback_param:      GTE ID
 * status:              An enum dma_status value. See dma.h for details.
 */
void tegra_gte_dma_complete_handler(void *callback_param, enum dma_status status);

/*
 * Initialize GTE
 *
 * Parameters:
 * id:                  GTE ID
 * occupancy_threshold: Generate an interrupt if the FIFO occupancy is larger
 *                      than this limit. Only valid for DMA is diabled.
 * irq_callback:        callback function for interrupt.
 * irq_data:            data context for interrupt.
 * dma_xfer:            DMA transfer fucntion
 * dma_en:              Enable DMA function
 *
 * Returns:
 * 0:                   Success
 * Non-zero:            Error
 */
int tegra_gte_setup(struct tegra_gte_id *id, uint32_t occupancy_threshold,
		    tegra_gte_callback irq_callback,
		    void *irq_data,
		    tegra_gte_dma_xfer dma_xfer,
		    bool dma_en);

/*
 * Enable events to be monitored
 */
void tegra_gte_slice_set_enable_mask(struct tegra_gte_id *id, uint8_t slice, uint32_t bitmap);

/*
 * GTE interrupt handler
 */
void tegra_gte_irq(void *data);

#endif
