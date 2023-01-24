/*
 * Copyright (c) 2019-2020, NVIDIA CORPORATION. All rights reserved.
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

#ifndef __CONFIG_T19X_H
#define __CONFIG_T19X_H

#include <tke-tegra-hw.h>

/* Source configuration */

#define PM_USE_HW_SEQUENCER_FOR_PLLAON	1
#define TIMER_CLK_SRC	TEGRA_TKE_CLK_SRC_USECCNT

#define SCRATCH_SCRATCH_14_COMB	0xc390414 /* SCRATCH + SCRATCH_SCRATCH_14_COMB */
#define SCRATCH_MAGIC_MASK_COMB	0xffff0000
#define SCRATCH_MAGIC_VAL_COMB	0x1c1c0000
#define SCRATCH_UART_MASK_COMB	0x0000ffff

/* Source configuration ends */

/* SoC and App specific includes */

#include <combined-uart.h>
/* includes end */

/* i2c-app configs */
#if defined(ENABLE_SPE_FOR_NX)
#define I2C_TEST_AON_BUS    tegra_i2c_id_i2c2
#define I2C_TEST_SLV_ADDR	0x19

#else
#define I2C_TEST_AON_BUS    tegra_i2c_id_i2c8
#define I2C_TEST_SLV_ADDR	0x1a
#endif

/* UART app port definition */
#if defined(ENABLE_UART_APP)
	#define UART_APP_PORT tegra_uart_id_uartg
#endif

#endif
