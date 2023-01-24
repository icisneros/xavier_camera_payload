/*
 * Copyright (c) 2016-2017, NVIDIA CORPORATION.  All rights reserved.
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

#include <tegra-ivc.h>
#include <ivc-channels.h>
#include <ivc-config.h>
#include <ivc-echo-task.h>

static struct tegra_ivc_channel ivc_echo_ch = {
	.write_header = (void *)IVC_ECHO_WRITE_HEADER,
	.read_header = (void *)IVC_ECHO_READ_HEADER,
	.nframes = IVC_ECHO_CH_NFRAMES,
	.frame_size = IVC_ECHO_CH_FRAME_SIZE,
	.notify_remote = hsp_ivc_notify_ccplex,
	.channel_group = IVC_ECHO_CH_ID,
};

static struct ivc_channel_ops ivc_echo_ops = {
	.init = ivc_echo_task_init,
	.notify = ivc_echo_task_ivc_notified,
	.init_complete = ivc_echo_task_ivc_inited,
};

static struct ivc_task_id ivc_echo_task_id = {
	.name = "echo task",
	.ivc_ch = &ivc_echo_ch,
	.ops = &ivc_echo_ops,
};

struct ivc_task_id *ivc_ccplex_task_ids[IVC_NUM_CCPLEX_CHS] = {
	&ivc_echo_task_id,
};
