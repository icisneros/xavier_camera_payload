/*
 * Copyright (c) 2019, NVIDIA CORPORATION.  All rights reserved.
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

#define I2C_TEST_SLAVE_DEV_ID	0xd1
#define I2C_TEST_SLAVE_ID_REG	0x0
#define I2C_TEST_SLV_ADDR	0x68

void i2c_test(void)
{
	uint8_t data = 0;
	int ret;
	uint8_t id_reg = I2C_TEST_SLAVE_ID_REG;
	struct tegra_i2c_xfer xfers[] = {
		{
			.i2c_addr = I2C_TEST_SLV_ADDR,
			.is_read = false,
			.buf = &id_reg,
			.count = 1,
		},
		{
			.i2c_addr = I2C_TEST_SLV_ADDR,
			.is_read = true,
			.buf = &data,
			.count = sizeof(data),
		},
	};

	ret = tegra_i2c_transfer(&I2C_TEST_AON_BUS, xfers, ARRAY_SIZE(xfers));
	if (ret) {
		printf("I2C transfer failed\r\n");
		return;
	}
	if (data != I2C_TEST_SLAVE_DEV_ID) {
		printf("I2C failed reading correct device id\r\n");
		return;
	}

	printf("I2C test successful, sendor ID: 0x%x\r\n", data);
}
