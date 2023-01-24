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

#ifndef _TEGRA_ADMA_H_
#define _TEGRA_ADMA_H_

#include <stdbool.h>
#include <stdint.h>

#include <dma.h>

/* The combination of devices to DMA between */
enum tegra_adma_transfer_direction {
	TEGRA_ADMA_DIR_MEM_TO_MEM,
	TEGRA_ADMA_DIR_AHUB_TO_MEM,
	TEGRA_ADMA_DIR_MEM_TO_AHUB,
	TEGRA_ADMA_DIR_AHUB_TO_AHUB,
};

/*
 * direction:		A legal enum tegra_adma_transfer_direction value.
 * rx_slave_request:	A HW-specific request select value.
 * tx_slave_request:	A HW-specific request select value.
 * enable_flow_ctrl:	Use [rt]x_slave_request or not.
 * burst_size:		The number of words to transfer in each burst. This
 * 			affects both RX and TX transfers.
 * source:		The address to read from.
 * source_addr_wrap:	The number of word transfers to make before wrapping
 * 			the source address. 0 represents no wrap. Legal values
 * 			are 2^n with n from 0 through 10.
 * target:		The address to write to.
 * target_addr_wrap:	The number of word transfers to make before wrapping
 * 			the target address.0 represents no wrap. Legal values
 * 			are 2^n with n from 0 through 10.
 * transfer_count:	The number of bytes to transfer. This value must be
 * 			word aligned.
 * callback:		A function to call once the transfer is complete.
 *			Callbacks are only valid for asynchronous transfers.
 * callback_data:	Data to pass to the callback function as context.
 * synchronous:		Execute the transfer synchronously, or not.
 * 			true: Wait for the transfer to complete and return the
 * 			  status.
 * 			false: Return immediately after initiating the
 * 			  transfer, and execute the supplied callback once the
 * 			  transfer completes.
 * timeout:		Timeout for synchronous transfers.
 */
struct tegra_adma_transfer {
	enum tegra_adma_transfer_direction direction;
	uint8_t rx_slave_request;
	uint8_t tx_slave_request;
	bool enable_flow_ctrl;
	uint32_t burst_size;
	void *source;
	uint32_t source_addr_wrap;
	void *target;
	uint32_t target_addr_wrap;
	uint32_t transfer_count;
	dma_callback *callback;
	void *callback_data;
	bool synchronous;
	uint32_t timeout;
};

/*
 * The identity of an ADMA controller instance.
 * Provided by HW-specific code.
 */
struct tegra_adma_id;

/*
 * Perform global initialization of the ADMA controller.
 *
 * Parameters:
 * id:		The DMA controller instance.
 *
 * Returns:
 * 0:		Success.
 * Negative:	Error.
 */
int tegra_adma_init(struct tegra_adma_id *id);

/*
 * Initialize the requested DMA channel.
 *
 * Parameters:
 * id:		The DMA controller instance.
 * chan:	The channel number.
 *
 * Returns:
 * 0:		Success.
 * Negative:	Error.
 */
int tegra_adma_channel_init(struct tegra_adma_id *id, int chan);

/*
 * Execute DMA transfer on a channel.
 *
 * For synchronous transfers, the transfer status will be returned.
 * For asynchronous transfers, the provided callback will be executed once
 * the transfer is complete.
 *
 * Only a single transfer may execute at once; transfers cannot be queued or
 * scheduled.
 *
 * Parameters:
 * id:		The DMA controller instance.
 * chan:	The channel number.
 * transfer:	The parameters of the transfer to perform.
 *
 * Returns:
 * An enum dma_status value. See dma.h for details.
 */
enum dma_status tegra_adma_transfer(struct tegra_adma_id *id, int chan,
	struct tegra_adma_transfer *transfer);

/*
 * Returns the number of bytes transferred by the most recent DMA transfer.
 *
 * This function may only be called after an asynchronous transfer has been
 * requested, and so long as the transfer has not been aborted.
 *
 * Parameters:
 * id:		The DMA controller instance.
 * chan:	The channel number.
 *
 * Returns:
 * Positive:	The number of bytes transferred.
 * Negative:	Error.
 */
int tegra_adma_get_count_xferd(struct tegra_adma_id *id, int chan);

/*
 * Abort the currently executing DMA transfer
 *
 * This function may only be called after an asynchronous transfer has been
 * requested, and so long as the transfer has not been aborted.
 *
 * Parameters:
 * id:		The DMA controller instance.
 * chan:	The channel number.
 *
 * Returns:
 * 0:		Success.
 * Negative:	Error.
 */
int tegra_adma_abort(struct tegra_adma_id *id, int chan);

#endif
