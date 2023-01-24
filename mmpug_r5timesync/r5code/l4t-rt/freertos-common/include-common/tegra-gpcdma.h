/*
* Copyright (c) 2015-2016 NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _TEGRA_GPCDMA_H_
#define _TEGRA_GPCDMA_H_

#include <stdbool.h>
#include <stdint.h>

#include <dma.h>

/* Supported DMA transfer combinations */
enum tegra_gpcdma_transfer_direction {
	TEGRA_GPCDMA_XFER_DIR_MEM_TO_IO,
	TEGRA_GPCDMA_XFER_DIR_IO_TO_MEM,
	TEGRA_GPCDMA_XFER_DIR_MEM_TO_MEM,
};

/* Supported IO bus widths for DMA transfers */
enum tegra_gpcdma_bus_width {
	TEGRA_GPCDMA_IO_BUS_WIDTH_8,
	TEGRA_GPCDMA_IO_BUS_WIDTH_16,
	TEGRA_GPCDMA_IO_BUS_WIDTH_32,
};

/*
 * direction:		A valid enum tegra_gpcdma_transfer_direction value.
 * bus_width:		IO buswidth. A valid enum tegra_gpcdma_bus_width value.
 * burst_size:		The number of words to transfer in each burst. Valid for
 * 			both tx/rx transfers.
 * src_addr:		The source address to read from.
 * src_addr_wrap:	The number of word transfers to make before wrapping
 * 			the src address. 0 represents no wrap.
 * 			Legal values for io addr wrap are 2^n with n from
 * 			0 through 6.
 * 			Legal values for mc addr wrap are 2^n with n from
 * 			5 through 11.
 * dst_addr:		The target address to write to.
 * dst_addr_wrap:	The number of word transfers to make before wrapping
 * 			the mc address. 0 represents no wrap.
 * 			Legal values for io addr wrap are 2^n with n from
 * 			0 through 6.
 * 			Legal values for mc addr wrap are 2^n with n from
 * 			5 through 11.
 * xfer_count:		The number of bytes to be transferred. This value must
 *			be word aligned.
 * en_flow_ctrl:	Use rx/tx slave request or not.
 * slave_req:		HW specific slave request select value. Used for both
 * 			rx/tx transfers.
 * synchronous:		Execute a synchronous transfer or not.
 * 			If true, wait till DMA transfer is done and return
 * 			status.
 * 			If false, return after starting DMA transfer and execute
 * 			the provided callback upon transfer completion.
 * timeout:		Timeout for synchronous transfers.
 * callback:		A function to execute once the transfer is done.
 * 			Callback is valid only for asynchronous transfers.
 * callback_param:	Data to be passed as context to the callback function.
 */
struct tegra_gpcdma_xfer {
	enum tegra_gpcdma_transfer_direction direction;
	enum tegra_gpcdma_bus_width bus_width;
	uint32_t burst_size;
	void *src_addr;
	uint32_t src_addr_wrap;
	void *dst_addr;
	uint32_t dst_addr_wrap;
	uint32_t xfer_count;
	bool en_flow_ctrl;
	uint8_t slave_req;
	bool synchronous;
	uint32_t timeout;
	dma_callback *callback;
	void *callback_param;
};

/* The identity of GPCDMA controller instance */
struct tegra_gpcdma_id;

/*
 * Perform global initialization of the GPCDMA controller.
 *
 * Parameters:
 * id:		The DMA controller instance.
 *
 * Returns:
 * 0:		Success.
 * Negative:	Error.
 */
int tegra_gpcdma_init(struct tegra_gpcdma_id *id);

/*
 * Initialize the requested DMA channel. If the channel is already initialized,
 * returns error.
 *
 * Parameters:
 * id:		GPCDMA controller instance.
 * chan_num:	Channel number.
 *
 * Returns:
 * 0: Success
 * Negative: error.
 */
int tegra_gpcdma_channel_init(struct tegra_gpcdma_id *id, int chan_num)
__attribute__((deprecated));

/*
 * De-initialize the requested DMA channel. If the channel is busy or not
 * initialized, return error.
 *
 * Parameters:
 * id:		GPCDMA controller instance.
 * chan_num:	Channel number.
 *
 * Returns:
 * 0: Success
 * Negative: error.
 */
int tegra_gpcdma_channel_deinit(struct tegra_gpcdma_id *id, int chan_num)
__attribute__((deprecated));

/*
 * Execute DMA transfer on the specified channel.
 *
 * For synchronous transfers, transfer status would be returned.
 * For asynchronous transfers, provided callback would be executed upon
 * transfer completion.
 *
 * This function will return error if the channel is not initialized or the
 * addresses are not aligned properly. No other parameter validity checks
 * will be performed.
 *
 * Parameters:
 * id:		DMA controller instance.
 * chan_num:	Channel number.
 * dma_xfer:	DMA transfer parameters.
 *
 * Returns:
 * An enum dma_status value. See dma.h for details.
 */
enum dma_status tegra_gpcdma_transfer(struct tegra_gpcdma_id *id,
	int chan_num, struct tegra_gpcdma_xfer *transfer);

/*
 * Returns the number of bytes transferred successfully using DMA.
 *
 * Parameters:
 * id:		DMA controller instance.
 * chan_num:	Channel number.
 *
 * Returns:
 * Positive: No of bytes transferred.
 * 0: No data transferred.
 */
int tegra_gpcdma_get_count_xferred(struct tegra_gpcdma_id *id, int chan_num);

/*
 * Abort the DMA transfer in progress.
 *
 * Parameters:
 * id:		DMA controller instance.
 * chan_num:	Channel number.
 */
int tegra_gpcdma_abort(struct tegra_gpcdma_id *id, int chan_num);

/*
 * GPCDMA IRQ handler. Call this whenever DMA controller raises an IRQ.
 * Each DMA channel has a separate IRQ number.
 *
 * Parameters:
 * ID:		DMA controller instance.
 * chan_num:	The channel number for which interrupt is raised.
 */
void tegra_gpcdma_irq(struct tegra_gpcdma_id *id, int chan_num);

#endif
