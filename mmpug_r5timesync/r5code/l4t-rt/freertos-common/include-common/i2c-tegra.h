/*
 * Copyright (c) 2014-2016 NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _I2C_TEGRA_H_
#define _I2C_TEGRA_H_

#include <stdbool.h>
#include <stdint.h>

struct tegra_i2c_id;

struct tegra_i2c_xfer {
	uint8_t i2c_addr;
	bool is_read;
	uint8_t *buf;
	uint32_t count;
};

int tegra_i2c_init(struct tegra_i2c_id *id, uint32_t bus_clkrate);
void tegra_i2c_fini(struct tegra_i2c_id *id);
uint32_t tegra_i2c_phys_addr(struct tegra_i2c_id *id);
int tegra_i2c_transfer(struct tegra_i2c_id *id, struct tegra_i2c_xfer *xfers, uint32_t num_xfers);
void tegra_i2c_irq(void *data);

int tegra_i2c_suspend(struct tegra_i2c_id *id);
int tegra_i2c_resume(struct tegra_i2c_id *id);

#endif
