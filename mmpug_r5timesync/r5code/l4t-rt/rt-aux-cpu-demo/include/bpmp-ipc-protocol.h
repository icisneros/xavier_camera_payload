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

#ifndef _BPMP_IPC_PROTOCOL_H_
#define _BPMP_IPC_PROTOCOL_H_

/*
 * The handshake protocol (light weight IPC) between BPMP and
 * SPE uses shared mail box, so that a different interrupt is
 * used for this version instead of doorbell interrupts. We
 * will be using two mail boxes, one for SPE -> BPMP and the
 * other for BPMP -> SPE
 */

/*
 *		SPE SHARED MAILBOX REGISTER: MBOX0
 *
 *			BPMP -> SPE
 *
 * When BPMP wishes to send a handshake message to SPE, it writes
 * the message to DATA fild of MBOX0 and sets the TAG field
 */

/*
 *		SPE SHARED MAILBOX REGISTER: MBOX1
 *
 *			SPE -> BPMP
 *
 * When SPE wishes to send a handshake message to SPE, it writes
 * the message to DATA fild of MBOX1 and sets the TAG field
 */

/*
 * The following macros indicate the message received.
 * As we are using two mailboxes we can use the same
 * message for ack while sending to BPMP
 */

/* MSG from SPE to BPMP */
#define BPMP_IPC_MSG_BPMP_PING_ACK		1
#define BPMP_IPC_MSG_DRAM_WAKE_REQ		2
#define BPMP_IPC_MSG_SPE_PING			3
#define BPMP_IPC_MSG_SC7_ENTRY_ACK		6
#define BPMP_IPC_MSG_SC7_EXIT_ACK		7
#define BPMP_IPC_MSG_ID_MAX			9

/* MSG from BPMP to SPE */
#define BPMP_IPC_MSG_BPMP_PING			1
#define BPMP_IPC_MSG_DRAM_WAKE_REQ_ACK		2
#define BPMP_IPC_MSG_SPE_PING_ACK		3
#define BPMP_IPC_MSG_SC7_ENTRY			6
#define BPMP_IPC_MSG_SC7_EXIT			7

#endif
