/*
 * Copyright (c) 2014-2016, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _IVC_TASK_H_
#define _IVC_TASK_H

#include <FreeRTOS.h>

struct ivc_task_id;

/*
 * Create and start an IVC echo task instance.
 *
 * Parameters:
 * id:		Configuration parameters for this task instance. This pointer
 *		must be valid for the duration of task execution.
 *
 * Returns:
 * NULL:	Error.
 * Other:	A task state handle, to be passed to other IVC echo task APIs.
 */
void *ivc_echo_task_init(struct ivc_task_id *id);

/*
 * Inform the IVC echo task that IVC state has changed.
 *
 * Parameters:
 * state:	The result from calling ivc_echo_task_init().
 *
 * Returns:
 * 0:		OK
 * Other:	Error.
 */
int ivc_echo_task_ivc_notified(void *state, BaseType_t *higher_prio_task_woken);

/*
 * Inform the IVC echo task that IVC RAM has been inited.
 *
 * Parameters:
 * state:	The result from calling ivc_echo_task_init().
 *
 * Returns:
 * 0:		OK
 * Other:	Error.
 */
int ivc_echo_task_ivc_inited(void *state, BaseType_t *higher_prio_task_woken);

#endif
