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

#ifndef _DMA_H_
#define _DMA_H_

/*
 * This header defines the part of the DMA API that is common between DMA
 * controller drivers.
 *
 * Since different DMA HW has different capabilities, the mechanism to control
 * a DMA driver (i.e. initialize the driver, and initiate transfers) differs
 * between drivers, and hence is not currently unified here.
 *
 * However, to simplify code that is portable across multiple DMA HW, certain
 * status codes and callback types are unified here.
 */

/*
 * The completion status of a DMA transfer.
 *
 * These values will be returned by DMA driver function which initiated DMA
 * transfers. For asynchronous transfers, the value returned represents the
 * status of submitting the request to the hardware. For synchronous
 * transfers, the value returned represents the overall status of the
 * transaction.
 *
 * For asynchronous transfers, these values will also be passed to the caller-
 * supplied completion callback.
 *
 *                Ret async    Ret sync    Async callback
 * NOT_INITIATED: Y            Y           N
 * EXECUTING:     Y            N           N
 * COMPLETE:      N            Y           Y
 * ABORTED:       N            N           Y
 * TIMEOUT:       N            Y           N
 */
enum dma_status {
	DMA_STATUS_NOT_INITIATED,
	DMA_STATUS_EXECUTING,
	DMA_STATUS_COMPLETE,
	DMA_STATUS_ABORTED,
	DMA_STATUS_TIMEOUT,
};

/*
 * The type of a function to call whenever a DMA transfer has completed.
 *
 * Parameters:
 * callback_param:	A copy of a value passed to the DMA driver function
 * 			that initiated the transfer. Clients may used this to
 * 			pass context to the DMA completion callback.
 * status:		Status of the transfer.
 */
typedef void dma_callback(void *callback_param, enum dma_status status);

#endif
