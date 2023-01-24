/*
 * Copyright (c) 2015-2017, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _TEGRA_IVC_H_
#define _TEGRA_IVC_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * Tegra IVC low-level IPC driver.
 *
 * Each IPC channel provides bi-directional communication between two entities
 * (typically CPUs).
 *
 * The overall channel is built from two separate uni-directional channels, one
 * for each direction of communication.
 *
 * Each of these uni-directional channels consists of a header and a data
 * buffer, arranged contiguously in memory. The header is divided into two
 * parts; one part written by the producer and read by the consumer, and the
 * other part written by the consumer and read by the producer. The data buffer
 * is divided into a fixed number of frames. These frames are used as a ring
 * buffer.
 *
 * Each part of the header and each data frame is an exact multiples of the
 * cache line size. If the cache line size differs on the CPUs, the larger
 * cache line size must be assumed.
 *
 * The two uni-directional channels may be located at arbitrary memory
 * locations; they need not be contiguous.
 *
 * Typically, inter-CPU interrupts are used to signal whenever relevant
 * information has changed, such as when a producer has produced more data that
 * the consumer should process, or the consumer has consumed data thus allowing
 * the producer to produce more.
 *
 * This library does not directly deal with these interrupts. Rather:
 * - Clients of the library are expected to call into the library whenever the
 *   remote CPU notifies the local CPU of a change.
 * - Whenever the library wishes to notify the remote CPU of a change, a
 *   callback is executed to do so. The client of the library must implement
 *   this callback as appropriate.
 *
 * This library performs all necessary cache management internally.
 *
 * This library never copies data into or out of the data buffers. Rather, it
 * provides pointers to those buffers to its client, and expects the client to
 * copy data as necessary, or perhaps fill/drain the buffers via DMA.
 */

struct tegra_ivc_channel;

/*
 * Callback from the tegra_ivc code to the host application indicating that
 * the remote CPU should be notified of a change to the channel.
 *
 * Parameters:
 * is_read:	True if the channel this CPU reads was updated. False if the
 * 		channel this CPU writes was updated.
 */
typedef int (*tegra_ivc_notify_remote)(struct tegra_ivc_channel *channel,
	bool is_read);

/*
 * Structure representing the local SW state associated with a channel.
 * Clients are expected to fill in the "configuration" fields before calling
 * any library function. The library manages the "Run-time state" fields
 * entirely internally; clients should not use or modify these fields.
 */
struct tegra_ivc_channel {
	// Configuration
	// The channel this CPU writes to
	void *write_header;
	// The channel this CPU reads from
	void *read_header;
	uint32_t nframes;
	uint32_t frame_size;
	tegra_ivc_notify_remote notify_remote;
	/* Group of channels activated by notify_remote() */
	uint32_t channel_group;

	// Run-time state
	uint32_t write_count;
	uint32_t read_count;
};

/*
 * Initialize a channel object's local SW state, and the IPC buffer memory.
 * This resets all channel headers in the IPC memory.
 *
 * Parameters:
 * ch:		The structure describing the channel
 *
 * Returns:
 * 0:		Success
 * Non-zero:	Error
 */
int tegra_ivc_init_channel_in_ram(struct tegra_ivc_channel *ch);

/*
 * Initialize a channel object's local SW state, from already initialized or
 * in-use IPC buffer memory.
 *
 * Parameters:
 * ch:		The structure describing the channel
 *
 * Returns:
 * 0:		Success
 * Non-zero:	Error
 */
int tegra_ivc_init_channel_from_ram(struct tegra_ivc_channel *ch);

/*
 * Validate channel config.
 */
bool tegra_ivc_check_channel(void *write_header,
			void *read_header,
			uint32_t nframes,
			uint32_t frame_size);

/*
 * Reset a channel object's local SW state using synchronization protocol.
 *
 * Parameters:
 * ch:		The structure describing the channel
 */
void tegra_ivc_channel_reset(struct tegra_ivc_channel *ch);

/*
 * Ensure a channel object is synchronized.
 *
 * Parameters:
 * ch:		The structure describing the channel
 *
 * Returns:
 * false:	Synchronization in progress
 * true:	Channel is synchronized
 */
bool tegra_ivc_channel_notified(struct tegra_ivc_channel *ch);

/*
 * Check if a channel object is synchronized.
 *
 * Parameters:
 * ch:		The structure describing the channel
 *
 * Returns:
 * false:	Synchronization in progress
 * true:	Channel is synchronized
 */
bool tegra_ivc_channel_is_synchronized(struct tegra_ivc_channel *ch);

/*
 * Query the number of frames that are available for reception.
 *
 * The count value returned by this function may change over time as the remote
 * CPU adds frames to the channel. However, it cannot normally decrease until
 * a call to tegra_ivc_rx_notify_buffers_consumed().
 *
 * Callers should call this function only when notified of a change by the
 * remote CPU, and cache the results. This function is relatively heavy-
 * weight since it invalidates caches.
 *
 * Parameters:
 * ch:		The structure describing the channel.
 *
  * Returns:
 * Zero:	No data available.
 * Positive:	The number of frames available.
 */
unsigned tegra_ivc_rx_get_read_available(struct tegra_ivc_channel *ch);

/*
 * Get a pointer to an RX frame.
 *
 * As IVC is a circular buffer, the available RX frames may not all be
 * contiguous in RAM. This function can be used to access multiple RX frames
 * without acknowledging the previous ones.
 *
 * If only the first (oldest) available RX frame or only contiguous RX frames
 * needs to be accessed at once, then
 * tegra_ivc_rx_get_contiguous_read_available() should be used instead.
 *
 * Note this function does not invalidate caches, nor perform any sanity check.
 * It must only be used in conjunction with tegra_ivc_rx_get_read_available(),
 * and the requested (zero-based) frame index must be strictly less than the
 * number of available frames. Otherwise, behaviour is undefined.
 *
 * To acknowledge buffers and free space for further RX frames,
 * tegra_ivc_rx_notify_buffers_consumed() must be called later.
 *
 * Parameters:
 * ch:		The structure describing the channel.
 * idx:		Frame index (0 is the oldest available RX frame)
 *
 * Returns:
 * A pointer to the beginning of the requested frame.
 */
void *tegra_ivc_rx_get_read_frame(struct tegra_ivc_channel *ch, unsigned n);

/*
 * Query the number of contiguous RX frames that are available for reception.
 *
 * The available frames may not all be contiguous in RAM; multiple calls
 * to tegra_ivc_tx_get_contiguous_read_available() may be required to obtain
 * all available frames pointers, and process them.
 *
 * The count value returned by this function may change over time as the remote
 * CPU adds frames to the channel.
 *
 * Callers should call this function only when notified of a change by the
 * remote CPU, and cache the results. This function is relatively heavy-
 * weight since it invalidates caches. The only exception is that when
 * *non_contig_available == true, this function should be called again to
 * retrieve a pointer to the wrapped buffers.
 *
 * The buffer pointer "returned" by this function will not advance unless
 * tegra_ivc_rx_notify_buffers_consumed() is called.
 *
 * Parameters:
 * ch:		The structure describing the channel.
 *
 * Updates:
 * *buffer_out:	To point at the first frame containing data to process.
 * *non_contig_available:
 * 		true if more frames are available at this time, but are not in
 * 		contiguous memory (i.e. the buffer wrapped).
 * 		false if the returned frame count is not limited by buffer
 * 		wrapping.
 *
 * Returns:
 * Zero:	No data available.
 * Positive:	The number of frames available.
 */
int tegra_ivc_rx_get_contiguous_read_available(struct tegra_ivc_channel *ch,
	const char **buffer_out, bool *non_contig_available);

/*
 * Query the RX frames that are available for reception.
 *
 * The count value returned by this function may change over time as the remote
 * CPU adds frames to the channel.
 *
 * Callers should call this function only when notified of a change by the
 * remote CPU, and cache the results. This function is relatively heavy-
 * weight since it invalidates caches. The only exception is that when
 * *non_contig_available == true, this function should be called again to
 * retrieve a pointer to the wrapped buffers.
 *
 * The buffer pointer "returned" by this function will not advance unless
 * tegra_ivc_rx_notify_buffers_consumed() is called.
 *
 * Parameters:
 * ch:		The structure describing the channel.
 * count:	The number of frames to return.
 *
 * Updates:
 * buffer_out[count]:	To point at the first frame containing data to process.
 *
 * Returns:
 * Zero:	No data available.
 * Positive:	The number of frames available (may be larger than count).
 */
int tegra_ivc_rx_get_frames(struct tegra_ivc_channel *ch,
		const char *buffers_out[], unsigned count);

/*
 * Notify the remote producer that frames have been consumed.
 *
 * Only contiguous frames can be marked consumed; each call to
 * tegra_ivc_rx_get_contiguous_read_available() should be paired with a number
 * of calls to tegra_ivc_rx_notify_buffers_consumed(), with the "count"
 * parameters adding up to tegra_ivc_rx_get_contiguous_read_available()'s
 * return value.
 *
 * Callers should generally batch up calls to this function when processing
 * multiple received frames, rather than repeatedly calling this function for
 * each frame. This function cleans caches; batching calls reduces the
 * overhead of the cache operations. Even so, throughput, latency, and
 * pipelining requirements might require calling this function more often than
 * once per call to tegra_ivc_rx_get_contiguous_read_available().
 *
 * Parameters:
 * ch:		The structure describing the channel.
 * count:	The number of frames consumed.
 *
 * Returns:
 * 0:		Success
 * Non-zero:	Error
 */
int tegra_ivc_rx_notify_buffers_consumed(struct tegra_ivc_channel *ch,
	int count);

/*
 * Query the number of frame buffers that are available for sending.
 *
 * The count value returned by this function may change over time as the remote
 * CPU frees used frame buffers. However, it cannot normally decrease until
 * a call to tegra_ivc_tx_send_buffers().
 *
 * Callers should call this function only when notified of a change by the
 * remote CPU, and cache the results. This function is relatively heavy-
 * weight since it invalidates caches.
 *
 * Parameters:
 * ch:		The structure describing the channel.
 *
  * Returns:
 * Zero:	No space available.
 * Positive:	The number of free frame buffers.
 */
unsigned tegra_ivc_tx_get_write_space(struct tegra_ivc_channel *ch);

/*
 * Get a pointer to an TX frame.
 *
 * As IVC is a circular buffer, the free TX frame buffes may not all be
 * contiguous in RAM. This function can be used to access multiple TX frame
 * buffers without committing the previous ones for sending.
 *
 * If only the first (oldest) available TX frame buffer or only contiguous TX
 * frame buffers need to be accessed at once, then
 * tegra_ivc_tx_get_contiguous_write_space() should be used instead.
 *
 * Note this function does not invalidate caches, nor perform any sanity check.
 * It must only be used in conjunction with tegra_ivc_tx_get_write_space(),
 * and the requested (zero-based) frame index must be strictly less than the
 * number of available frames. Otherwise, behaviour is undefined.
 *
 * Parameters:
 * ch:		The structure describing the channel.
 * idx:		Frame index (0 is the oldest available RX frame)
 *
 * Returns:
 * A pointer to the beginning of the requested frame.
 */
void *tegra_ivc_tx_get_write_buffer(struct tegra_ivc_channel *ch, unsigned n);

/*
 * Query the number of contiguous TX frames that are available for filling.
 *
 * The available frames may not all be contiguous in RAM; multiple calls
 * to tegra_ivc_tx_get_contiguous_write_space() may be required to obtain
 * all available frame pointers, and fill them.
 *
 * The count value returned by this function may change over time as the remote
 * CPU consumes frames from the channel.
 *
 * Callers should call this function only when notified of a change by the
 * remote CPU, and cache the results. This function is relatively heavy-
 * weight since it invalidates caches.The only exception is that when
 * *non_contig_available == true, this function could be called again to
 * retrieve a pointer to the wrapped buffers.
 *
 * The buffer pointer "returned" by this function will not advance unless
 * tegra_ivc_tx_send_buffers() is called.
 *
 * Parameters:
 * ch:		The structure describing the channel.
 *
 * Updates:
 * *buffer_out:	To point at the first frame into which data should be placed.
 * *non_contig_available:
 * 		true if more frames are available at this time, but are not in
 * 		contiguous memory (i.e. the buffer wrapped).
 * 		false if the returned frame count is not limited by buffer
 * 		wrapping.
 *
 * Returns:
 * Zero:	No data available.
 * Positive:	The number of empty frames that may be filled.
 */
int tegra_ivc_tx_get_contiguous_write_space(struct tegra_ivc_channel *ch,
	char **buffer_out, bool *non_contig_available);

/*
 * Query the number of TX frames that are available for filling.
 *
 * The available frames may not all be contiguous in RAM.
 *
 * The count value returned by this function may change over time as the remote
 * CPU consumes frames from the channel.
 *
 * Callers should call this function only when notified of a change by the
 * remote CPU, and cache the results. This function is relatively heavy-
 * weight since it invalidates caches.
 *
 * The buffer pointers "returned" by this function will not advance unless
 * tegra_ivc_tx_send_buffers() is called.
 *
 * Parameters:
 * ch:		The structure describing the channel.
 * count:	The number of frames to return.
 *
 * Updates:
 * buffers_out[count]:	The frames into which data could be placed.
 *
 * Returns:
 * Zero:	No data available.
 * Positive:	The number of empty frames that may be filled.
 */
int tegra_ivc_tx_get_frames(struct tegra_ivc_channel *ch,
		char **buffers_out, unsigned count);

/*
 * Notify the remote consumer that frames have been produced.
 *
 * Only contiguous frames can be marked produced; each call to
 * tegra_ivc_tx_get_contiguous_write_space() should be paired with a number
 * of calls to tegra_ivc_tx_send_buffers(), with the "count"
 * parameters adding up to tegra_ivc_tx_get_contiguous_write_space()'s
 * return value.
 *
 * Callers should generally batch up calls to this function when producing
 * multiple frames, rather than repeatedly calling this function for
 * each frame. This function cleans caches; batching calls reduces the
 * overhead of the cache operations. Even so, throughput, latency, and
 * pipelining requirements might require calling this function more often than
 * once per call to tegra_ivc_tx_get_contiguous_write_space().
 *
 * Parameters:
 * ch:		The structure describing the channel.
 * count:	The number of frames consumed.
 *
 * Returns:
 * 0:		Success
 * Non-zero:	Error
 */
int tegra_ivc_tx_send_buffers(struct tegra_ivc_channel *ch, int count);

#endif
