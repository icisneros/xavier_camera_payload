/*
 * Copyright (c) 2015-2018, NVIDIA CORPORATION.  All rights reserved.
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

#include <stddef.h>
#include <string.h>

#include <cache.h>
#include <barriers.h>
#include <tegra-ivc.h>

/*
 * FIXME: We may want to parameterize this in the future
 * FIXME: We may want to ensure this is a multiple of the runtime cache line
 *        size in the future.
 */
#define TEGRA_IVC_ALIGN 64

/*
 * IVC channel reset protocol.
 *
 * Each end uses its tx_channel.state to indicate its synchronization state.
 * Declaring as anonymous enum so that the enum values can be compared directly
 * with int32_t without any explicit conversion.
 */
enum {
	/*
	 * This value is zero for backwards compatibility with services that
	 * assume channels to be initially zeroed. Such channels are in an
	 * initially valid state, but cannot be asynchronously reset, and must
	 * maintain a valid state at all times.
	 *
	 * The transmitting end can enter the established state from the sync or
	 * ack state when it observes the receiving endpoint in the ack or
	 * established state, indicating that has cleared the counters in our
	 * rx_channel.
	 */
	ivc_state_established = 0,

	/*
	 * If an endpoint is observed in the sync state, the remote endpoint is
	 * allowed to clear the counters it owns asynchronously with respect to
	 * the current endpoint. Therefore, the current endpoint is no longer
	 * allowed to communicate.
	 */
	ivc_state_sync,

	/*
	 * When the transmitting end observes the receiving end in the sync
	 * state, it can clear the w_count and r_count and transition to the ack
	 * state. If the remote endpoint observes us in the ack state, it can
	 * return to the established state once it has cleared its counters.
	 */
	ivc_state_ack
};

/*
 * This structure is divided into two-cache aligned parts, the first is only
 * written through the write_header pointer, while the second is only written
 * through the read_header pointer. This delineates ownership of the cache lines,
 * which is critical to performance and necessary in non-cache coherent
 * implementations.
 */
struct tegra_ivc_channel_header {
	union {
		struct {
			uint32_t count;
			int32_t state;
		} d;
		uint8_t align[TEGRA_IVC_ALIGN];
	} write;
	union {
		struct {
			uint32_t count;
		} d;
		uint8_t align[TEGRA_IVC_ALIGN];
	} read;
};

static inline int tegra_ivc_notify_read_remote(struct tegra_ivc_channel *ch)
{
	return ch->notify_remote(ch, true);
}

static inline int tegra_ivc_notify_write_remote(struct tegra_ivc_channel *ch)
{
	return ch->notify_remote(ch, false);
}

static size_t tegra_ivc_total_channel_size(struct tegra_ivc_channel *ch)
{
	return sizeof(struct tegra_ivc_channel_header) +
		(ch->nframes * ch->frame_size);
}

static void *tegra_ivc_pointer_after(struct tegra_ivc_channel *ch, void *header)
{
	uint8_t *p = header;
	p += tegra_ivc_total_channel_size(ch);
	return p;
}

/* Invalidate an entire channel header (both read and write sub-structs) */
static void tegra_ivc_invalidate_header(struct tegra_ivc_channel_header *hdr)
{
	cache_invalidate(hdr, sizeof(*hdr));
}

/*
 * Invalidate the entire channel header for our write path. This is required
 * before we start communication over a channel for which the remote CPU has
 * already set up the headers, so that we're completely in sync on both read
 * and write counts.
 */
static void tegra_ivc_invalidate_entire_write_header(
	struct tegra_ivc_channel *ch)
{
	tegra_ivc_invalidate_header(ch->write_header);
}

/*
 * Invalidate the entire channel header for our read path. This is required
 * before we start communication over a channel for which the remote CPU has
 * already set up the headers, so that we're completely in sync on both read
 * and write counts.
 */
static void tegra_ivc_invalidate_entire_read_header(
	struct tegra_ivc_channel *ch)
{
	tegra_ivc_invalidate_header(ch->read_header);
}

/*
 * Invalidate the remote portion of the header for our write path.
 * That is, the read sub-struct of the write path header.
 */
static void tegra_ivc_invalidate_write_path_remote_header(
	struct tegra_ivc_channel *ch)
{
	struct tegra_ivc_channel_header *hdr = ch->write_header;

	cache_invalidate(&hdr->read.d, sizeof(hdr->read.d));
}

/*
 * Invalidate the remote portion of the header for our read path.
 * That is, the write sub-struct of the read path header.
 */
static void tegra_ivc_invalidate_read_path_remote_header(
	struct tegra_ivc_channel *ch)
{
	struct tegra_ivc_channel_header *hdr = ch->read_header;

	cache_invalidate(&hdr->write.d, sizeof(hdr->write.d));
}

/*
 * Clean the local portion of the header for our write path.
 * That is, the write sub-struct of the write path header.
 */
static void tegra_ivc_clean_write_path_local_header(struct tegra_ivc_channel *ch)
{
	struct tegra_ivc_channel_header *hdr = ch->write_header;

	cache_clean(&hdr->write.d, sizeof(hdr->write.d));
}

/*
 * Clean the local portion of the header for our read path.
 * That is, the read sub-struct of the read path header.
 */
static void tegra_ivc_clean_read_path_local_header(struct tegra_ivc_channel *ch)
{
	struct tegra_ivc_channel_header *hdr = ch->read_header;

	cache_clean(&hdr->read.d, sizeof(hdr->read.d));
}

static int tegra_ivc_validate_channel_conf(struct tegra_ivc_channel *ch)
{
	if (offsetof(struct tegra_ivc_channel_header, write) &
			(TEGRA_IVC_ALIGN - 1))
		return -1;
	if (offsetof(struct tegra_ivc_channel_header, read) &
			(TEGRA_IVC_ALIGN - 1))
		return -1;
	if (sizeof(struct tegra_ivc_channel_header) & (TEGRA_IVC_ALIGN - 1))
		return -1;

	if (((size_t)(ch->write_header)) & (TEGRA_IVC_ALIGN - 1))
		return -1;
	if (((size_t)(ch->read_header)) & (TEGRA_IVC_ALIGN - 1))
		return -1;

	if (ch->frame_size & (TEGRA_IVC_ALIGN - 1))
		return -1;

	if (ch->write_header < ch->read_header) {
		if (tegra_ivc_pointer_after(ch, ch->write_header) >
				ch->read_header)
			return -1;
	} else {
		if (tegra_ivc_pointer_after(ch, ch->read_header) >
				ch->write_header)
			return -1;
	}

	return 0;
}

int tegra_ivc_init_channel_in_ram(struct tegra_ivc_channel *ch)
{
	size_t ch_size;

	int ret = tegra_ivc_validate_channel_conf(ch);
	if (ret)
		return ret;

	ch_size = tegra_ivc_total_channel_size(ch);
	memset(ch->write_header, 0, ch_size);
	memset(ch->read_header, 0, ch_size);

	cache_clean(ch->write_header, ch_size);
	cache_clean(ch->read_header, ch_size);

	ch->write_count = 0;
	ch->read_count = 0;

	return 0;
}

int tegra_ivc_init_channel_from_ram(struct tegra_ivc_channel *ch)
{
	struct tegra_ivc_channel_header *wh = ch->write_header;
	struct tegra_ivc_channel_header *rh = ch->read_header;

	int ret = tegra_ivc_validate_channel_conf(ch);
	if (ret)
		return ret;

	tegra_ivc_invalidate_entire_write_header(ch);
	tegra_ivc_invalidate_entire_read_header(ch);

	ch->write_count = wh->write.d.count;
	ch->read_count = rh->read.d.count;

	return 0;
}

void tegra_ivc_channel_reset(struct tegra_ivc_channel *ch)
{
	struct tegra_ivc_channel_header *tx_hdr = ch->write_header;

	tx_hdr->write.d.state = ivc_state_sync;

	tegra_ivc_clean_write_path_local_header(ch);
	barrier_memory_order();
	tegra_ivc_notify_write_remote(ch);
}

bool tegra_ivc_check_channel(void *write_header,
			void *read_header,
			uint32_t nframes,
			uint32_t frame_size)
{
	struct tegra_ivc_channel check = {
		.write_header = write_header,
		.read_header = read_header,
		.nframes = nframes,
		.frame_size = frame_size,
	};

	return tegra_ivc_validate_channel_conf(&check) == 0;
}

/*
 * ===============================================================
 *  IVC State Transition Table - see tegra_ivc_channel_notified()
 * ===============================================================
 *
 *	local	remote	action
 *	-----	------	-----------------------------------
 *	SYNC	EST	<none>
 *	SYNC	ACK	reset counters; move to EST; notify
 *	SYNC	SYNC	reset counters; move to ACK; notify
 *	ACK	EST	move to EST; notify
 *	ACK	ACK	move to EST; notify
 *	ACK	SYNC	reset counters; move to ACK; notify
 *	EST	EST	<none>
 *	EST	ACK	<none>
 *	EST	SYNC	reset counters; move to ACK; notify
 *
 * ===============================================================
 */

bool tegra_ivc_channel_notified(struct tegra_ivc_channel *ch)
{
	struct tegra_ivc_channel_header *tx_hdr = ch->write_header;
	struct tegra_ivc_channel_header *rx_hdr = ch->read_header;
	int32_t peer_state;

	/* Copy the receiver's state out of shared memory. */
	tegra_ivc_invalidate_read_path_remote_header(ch);
	peer_state = rx_hdr->write.d.state;

	if (peer_state == ivc_state_sync) {
		/*
		 * Order observation of ivc_state_sync before stores clearing
		 * tx_hdr.
		 */
		barrier_memory_order();

		/*
		 * Reset tx_hdr counters. The remote end is in the SYNC
		 * state and won't make progress until we change our state,
		 * so the counters are not in use at this time.
		 */
		tx_hdr->write.d.count = 0;
		rx_hdr->read.d.count = 0;

		ch->write_count = 0;
		ch->read_count = 0;

		tegra_ivc_clean_read_path_local_header(ch);

		/*
		 * Ensure that counters appear cleared before new state can be
		 * observed.
		 */
		barrier_memory_order();

		/*
		 * Move to ACK state. We have just cleared our counters, so it
		 * is now safe for the remote end to start using these values.
		 */
		tx_hdr->write.d.state = ivc_state_ack;
		tegra_ivc_clean_write_path_local_header(ch);

		/*
		 * Notify remote end to observe state transition.
		 */
		tegra_ivc_notify_write_remote(ch);

	} else if ((tx_hdr->write.d.state == ivc_state_sync) &&
			(peer_state == ivc_state_ack)) {
		/*
		 * Order observation of ivc_state_sync before stores clearing
		 * tx_hdr.
		 */
		barrier_memory_order();

		/*
		 * Reset tx_hdr counters. The remote end is in the ACK
		 * state and won't make progress until we change our state,
		 * so the counters are not in use at this time.
		 */
		tx_hdr->write.d.count = 0;
		rx_hdr->read.d.count = 0;

		ch->write_count = 0;
		ch->read_count = 0;

		tegra_ivc_clean_read_path_local_header(ch);

		/*
		 * Ensure that counters appear cleared before new state can be
		 * observed.
		 */
		barrier_memory_order();

		/*
		 * Move to ESTABLISHED state. We know that the remote end has
		 * already cleared its counters, so it is safe to start
		 * writing/reading on this channel.
		 */
		tx_hdr->write.d.state = ivc_state_established;

		tegra_ivc_clean_write_path_local_header(ch);

		/*
		 * Notify remote end to observe state transition.
		 */
		tegra_ivc_notify_write_remote(ch);

	} else if (tx_hdr->write.d.state == ivc_state_ack) {
		/*
		 * At this point, we have observed the peer to be in either
		 * the ACK or ESTABLISHED state. Next, order observation of
		 * peer state before storing to tx_hdr.
		 */

		barrier_memory_order();

		/*
		 * Move to ESTABLISHED state. We know that we have previously
		 * cleared our counters, and we know that the remote end has
		 * cleared its counters, so it is safe to start writing/reading
		 * on this channel.
		 */
		tx_hdr->write.d.state = ivc_state_established;

		tegra_ivc_clean_write_path_local_header(ch);

		/*
		 * Notify remote end to observe state transition.
		 */
		tegra_ivc_notify_write_remote(ch);

	} else {
		/*
		 * There is no need to handle any further action. Either the
		 * channel is already fully established, or we are waiting for
		 * the remote end to catch up with our current state. Refer
		 * to the diagram in "IVC State Transition Table" above.
		 */
	}

	return tx_hdr->write.d.state == ivc_state_established;
}

bool tegra_ivc_channel_is_synchronized(struct tegra_ivc_channel *ch)
{
	struct tegra_ivc_channel_header *tx_hdr;
	struct tegra_ivc_channel_header *rx_hdr;
	int32_t peer_state;

	if (ch == NULL)
		return false;

	tx_hdr = ch->write_header;
	rx_hdr = ch->read_header;

	tegra_ivc_invalidate_read_path_remote_header(ch);
	peer_state = rx_hdr->write.d.state;

	return peer_state == ivc_state_established &&
		tx_hdr->write.d.state == ivc_state_established;
}

static void *tegra_ivc_frame_pointer(struct tegra_ivc_channel *ch,
	struct tegra_ivc_channel_header *hdr, uint32_t frame)
{
	return (void *)(((char *)(hdr + 1)) + (ch->frame_size * frame));
}

static void *tegra_ivc_rx_get_raw_read_available(
	struct tegra_ivc_channel *ch, unsigned *total, unsigned *wrapped)
{
	struct tegra_ivc_channel_header *hdr = ch->read_header;
	uint32_t remote_write_count, avail;
	void *buf = NULL;

	tegra_ivc_invalidate_read_path_remote_header(ch);

	remote_write_count = hdr->write.d.count;
	/*
	 * There is a control dependency from the read value of
	 * remote_write_count to the calculation of avail to the decision by
	 * the caller whether to read from a buffer at all. Hence, no barrier
	 * is required here.
	 */
	avail = remote_write_count - ch->read_count;
	*total = avail;
	*wrapped = 0;

	if (avail > 0) {
		uint32_t index = ch->read_count % ch->nframes;

		if ((index + avail) > ch->nframes) {
			*wrapped = (index + avail) - ch->nframes;
			avail = ch->nframes - index;
		}

		buf = tegra_ivc_frame_pointer(ch, hdr, index);
		cache_invalidate(buf, avail * ch->frame_size);
	}

	return buf;
}

unsigned tegra_ivc_rx_get_read_available(struct tegra_ivc_channel *ch)
{
	struct tegra_ivc_channel_header *hdr = ch->read_header;
	unsigned avail, wrap;

	tegra_ivc_rx_get_raw_read_available(ch, &avail, &wrap);

	if (wrap > 0)
		cache_invalidate(hdr + 1, wrap * ch->frame_size);

	return avail;
}

void *tegra_ivc_rx_get_read_frame(struct tegra_ivc_channel *ch, unsigned n)
{
	uint32_t idx = (ch->read_count + n) % ch->nframes;

	return tegra_ivc_frame_pointer(ch, ch->read_header, idx);
}

int tegra_ivc_rx_get_contiguous_read_available(struct tegra_ivc_channel *ch,
	const char **buffer_out, bool *non_contig_available)
{
	unsigned total, wrap;

	*buffer_out = tegra_ivc_rx_get_raw_read_available(ch, &total, &wrap);
	*non_contig_available = wrap > 0;
	return total - wrap;
}

int tegra_ivc_rx_get_frames(struct tegra_ivc_channel *ch,
	const char *buffers_out[], unsigned count)
{
	struct tegra_ivc_channel_header *hdr = ch->read_header;
	uint32_t remote_write_count, count_available;
	uint32_t read_index;
	unsigned i;
	char *buf;

	tegra_ivc_invalidate_read_path_remote_header(ch);

	remote_write_count = hdr->write.d.count;
	/*
	 * There is a control dependency from the read value of
	 * remote_write_count to the calculation of count_available to the
	 * decision by the caller whether to read from a buffer at all. Hence,
	 * no barrier is required here.
	 */
	count_available = remote_write_count - ch->read_count;
	if (count_available < count)
		return 0;

	read_index = ch->read_count % ch->nframes;

	for (i = 0; i < count; i++) {
		buf = tegra_ivc_frame_pointer(ch, hdr, read_index++);
		cache_invalidate(buf, ch->frame_size);
		buffers_out[i] = buf;
		read_index %= ch->nframes;
	}

	return count_available;
}

int tegra_ivc_rx_notify_buffers_consumed(struct tegra_ivc_channel *ch,
	int count)
{
	struct tegra_ivc_channel_header *hdr = ch->read_header;
	uint32_t read_index;

	read_index = ch->read_count % ch->nframes;
	if ((read_index + count) > ch->nframes)
		return -1;

	/*
	 * Ensure local reads/writes of/to the buffer complete before remote
	 * CPU's fills
	 */
	barrier_memory_order();

	ch->read_count += count;
	hdr->read.d.count = ch->read_count;
	tegra_ivc_clean_read_path_local_header(ch);

	if (tegra_ivc_rx_get_read_available(ch) == ch->nframes - count)
		return tegra_ivc_notify_read_remote(ch);
	else
		return 0;
}

static void *tegra_ivc_tx_get_raw_write_space(
	struct tegra_ivc_channel *ch, unsigned *total, unsigned *wrapped)
{
	struct tegra_ivc_channel_header *hdr = ch->write_header;
	uint32_t remote_read_count, avail;
	void *buf = NULL;

	tegra_ivc_invalidate_write_path_remote_header(ch);

	remote_read_count = hdr->read.d.count;
	/*
	 * There is a control dependency from the read value of
	 * remote_write_count to the calculation of avail to the decision by
	 * the caller whether to read from a buffer at all. Hence, no barrier
	 * is required here.
	 */
	avail = ch->nframes - (ch->write_count - remote_read_count);
	*total = avail;
	*wrapped = 0;

	if (avail > 0) {
		uint32_t index = ch->write_count % ch->nframes;

		if ((index + avail) > ch->nframes)
			*wrapped = (index + avail) - ch->nframes;

		buf = tegra_ivc_frame_pointer(ch, hdr, index);
	}

	return buf;
}

unsigned tegra_ivc_tx_get_write_space(struct tegra_ivc_channel *ch)
{
	unsigned avail, wrap;

	tegra_ivc_tx_get_raw_write_space(ch, &avail, &wrap);
	return avail;
}

void *tegra_ivc_tx_get_write_buffer(struct tegra_ivc_channel *ch, unsigned n)
{
	uint32_t idx = (ch->write_count + n) % ch->nframes;

	return tegra_ivc_frame_pointer(ch, ch->write_header, idx);
}

int tegra_ivc_tx_get_contiguous_write_space(struct tegra_ivc_channel *ch,
	char **buffer_out, bool *non_contig_available)
{
	unsigned total, wrap;

	*buffer_out = tegra_ivc_tx_get_raw_write_space(ch, &total, &wrap);
	*non_contig_available = wrap > 0;
	return total - wrap;
}

int tegra_ivc_tx_get_frames(struct tegra_ivc_channel *ch,
		char *buffers_out[], unsigned count)
{
	struct tegra_ivc_channel_header *hdr = ch->write_header;
	uint32_t remote_read_count, count_available;
	uint32_t write_index;
	unsigned i;

	tegra_ivc_invalidate_write_path_remote_header(ch);

	remote_read_count = hdr->read.d.count;
	/*
	 * There is a control dependency from the read value of
	 * remote_read_count to the calculation of count_available to the
	 * decision by the caller whether to write to a buffer at all. Hence,
	 * no barrier is required here.
	 */
	count_available = ch->nframes - (ch->write_count - remote_read_count);
	if (count_available < count)
		return 0;

	write_index = ch->write_count % ch->nframes;

	for (i = 0; i < count; i++) {
		buffers_out[i] = tegra_ivc_frame_pointer(ch, hdr, write_index++);
		write_index %= ch->nframes;
	}

	return count_available;
}

int tegra_ivc_tx_send_buffers(struct tegra_ivc_channel *ch, int count)
{
	struct tegra_ivc_channel_header *hdr = ch->write_header;
	uint32_t write_index;
	char *write_buffer;

	write_index = ch->write_count % ch->nframes;
	if ((write_index + count) > ch->nframes)
		return -1;

	write_buffer = tegra_ivc_frame_pointer(ch, hdr, write_index);
	cache_clean(write_buffer, count * ch->frame_size);

	ch->write_count += count;
	hdr->write.d.count = ch->write_count;
	tegra_ivc_clean_write_path_local_header(ch);

	if (tegra_ivc_tx_get_write_space(ch) == ch->nframes - count)
		return tegra_ivc_notify_write_remote(ch);
	else
		return 0;
}
