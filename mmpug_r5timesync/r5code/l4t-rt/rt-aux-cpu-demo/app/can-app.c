/*
 * Copyright (c) 2019-2020, NVIDIA CORPORATION.  All rights reserved.
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

#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>
#include <tegra-can.h>
#include <tegra-mttcan.h>
#include <can-tegra-hw.h>

#include "can-app.h"
#include "can-app-rx-task.h"
#include "can-app-tx-task.h"

/*
 * Based on the platform, the app can be configured as either loopback between
 * both CAN controllers or as stand alone CAN transmitter or receiver. If
 * configured as CAN transmitter, CAN driver will create transmit element on
 * every transmit or bus state change, the app will display such elements as
 * well for debug purposes. Follow can-app-tx-task.c for more information.
 *
 * Follow can-app.md for more information on supported platforms and wiring
 * details.
 */

#define CAN0_CONTROLLER		&tegra_gttcan[0]
#define CAN1_CONTROLLER		&tegra_gttcan[1]

/* Configure appropriate CAN controllers as tx or rx */
#define ENABLE_CAN0_AS_TX
#define ENABLE_CAN1_AS_RX

/* Config sanity checks */
#if defined(ENABLE_CAN0_AS_TX) && defined(ENABLE_CAN0_AS_RX)
#error CAN0 can not be both TX and RX
#endif

#if defined(ENABLE_CAN1_AS_TX) && defined(ENABLE_CAN1_AS_RX)
#error CAN1 can not be both TX and RX
#endif

/*
 * Generally, CAN0 and CAN1 both can be TX or RX at the same time, but
 * the way demo application is designed; check and complain if both
 * the CAN controllers are configured same.
 */
#if defined(ENABLE_CAN0_AS_TX) && defined(ENABLE_CAN1_AS_TX)
#error CAN0 and CAN1 can not be both TX
#endif

#if defined(ENABLE_CAN0_AS_RX) && defined(ENABLE_CAN1_AS_RX)
#error CAN0 and CAN1 can not be both RX
#endif

/* Set appropriate CAN TX controller based on the ENABLE_CAN*_AS_TX define */
#if defined(ENABLE_CAN_TX)

#if defined(ENABLE_CAN0_AS_TX)
#define CAN_TX_CONTROLLER	CAN0_CONTROLLER
#elif defined(ENABLE_CAN1_AS_TX)
#define CAN_TX_CONTROLLER	CAN1_CONTROLLER
#else
#error Invalid CAN TX option.
#endif

#else
#define CAN_TX_CONTROLLER	NULL
#endif

/* Set appropriate CAN RX controller based on the ENABLE_CAN*_AS_RX define */
#if defined(ENABLE_CAN_RX)

#if defined(ENABLE_CAN0_AS_RX)
#define CAN_RX_CONTROLLER	CAN0_CONTROLLER
#elif defined(ENABLE_CAN1_AS_RX)
#define CAN_RX_CONTROLLER	CAN1_CONTROLLER
#else
#error Invalid CAN RX option.
#endif

#else
#define CAN_RX_CONTROLLER	NULL
#endif

void can_app_init(void)
{
	can_app_tx_task_init(CAN_TX_CONTROLLER);
	can_app_rx_task_init(CAN_RX_CONTROLLER);
}
