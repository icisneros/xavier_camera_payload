/*
 * Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _IVC_CONFIG_
#define _IVC_CONFIG_

#define AST_IVC_REGION		2
#define AST_MIN_CARVEOUT_SIZE	(1 << 12)

#define AON_STREAMID	22

/* Shared Semaphore Index for the IVC Carveout base and size */
#define IVC_CARVEOUT_BASE_SS_INDEX	0
#define IVC_CARVEOUT_SIZE_SS_INDEX	1

#define CCPLEX_CARVEOUT_BASE	0x80000000

#define TEGRA_IVC_ALIGN	64

#define IVC_NUM_CCPLEX_CHS	1

#define SMBOX_IVC_CHAN_SHIFT	16
#define SMBOX_IVC_NOTIFY_MASK	0xFFFF

#define IVC_CH_HDRS_SIZE	(TEGRA_IVC_ALIGN * 2)

#define IVC_CH_SIZE(nframes, frame_size) \
	(IVC_CH_HDRS_SIZE + (nframes * frame_size))

#define SMBOX_IVC_READY_MSG	0x2AAA5555
#define	SMBOX_IVC_NOTIFY	0x0000AABB

#define IVC_ECHO_TX_OFFSET	0
#define IVC_ECHO_RX_OFFSET	0x10000
#define IVC_ECHO_CH_ID		0
#define IVC_ECHO_WRITE_HEADER	(CCPLEX_CARVEOUT_BASE + IVC_ECHO_TX_OFFSET)
#define IVC_ECHO_READ_HEADER	(CCPLEX_CARVEOUT_BASE + IVC_ECHO_RX_OFFSET)
#define IVC_ECHO_CH_NFRAMES	16
#define IVC_ECHO_CH_FRAME_SIZE	(TEGRA_IVC_ALIGN * 1)
#define IVC_ECHO_CH_SIZE	IVC_CH_SIZE(IVC_ECHO_CH_NFRAMES, \
					IVC_ECHO_CH_FRAME_SIZE)
#define IVC_ECHO_CH_AFTER	IVC_ECHO_CH_SIZE

#define IVC_NOTIFY_TX_SS        2

#endif
