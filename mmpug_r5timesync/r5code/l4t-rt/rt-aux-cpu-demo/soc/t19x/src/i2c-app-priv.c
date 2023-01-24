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

#include <stdio.h>
#include <string.h>
#include <macros.h>
#include <i2c-tegra-hw.h>
#include "config.h"
#include "i2c-app-priv.h"

void i2c_test(void)
{
	int ret;
	uint8_t data_to_read[] = {0xff, 0xff};
#if defined(ENABLE_SPE_FOR_NX)
	uint8_t data_to_compare[] = {0x07, 0x07};
	uint8_t id_reg[] = {0x20};
#else
	uint8_t data_to_compare[] = {0x63, 0x11};
	uint8_t id_reg[] = {0x00, 0xff};
#endif
	struct tegra_i2c_xfer xfers[] = {
		{
			.i2c_addr = I2C_TEST_SLV_ADDR,
			.is_read = false,
			.buf = id_reg,
			.count = ARRAY_SIZE(id_reg),
		},
		{
			.i2c_addr = I2C_TEST_SLV_ADDR,
			.is_read = true,
			.buf = data_to_read,
			.count = ARRAY_SIZE(data_to_read),
		},
	};

	ret = tegra_i2c_transfer(&I2C_TEST_AON_BUS, xfers, ARRAY_SIZE(xfers));
	if (!ret){
		if (!memcmp(data_to_read, data_to_compare, 2)){
			printf("I2C test successful\r\n");
			return;
		}
	}
	printf("I2C failed reading id reg\r\n");
}
