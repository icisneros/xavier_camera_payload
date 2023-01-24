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

#ifndef _TEGRA186_GPIO_HW_H_
#define _TEGRA186_GPIO_HW_H

#include <tegra186-gpio.h>

/* Main GPIO controller */
extern struct tegra_gpio_id tegra186_gpio_id_main;
#define tegra_gpio_ops_main tegra186_gpio_ops
#define tegra_gpio_id_main tegra186_gpio_id_main

/* AON GPIO controller */
extern struct tegra_gpio_id tegra186_gpio_id_aon;
#define tegra_gpio_ops_aon tegra186_gpio_ops
#define tegra_gpio_id_aon tegra186_gpio_id_aon

/* GPIOs implemented by main GPIO controller */
enum {
	TEGRA_GPIO_BANK_ID_A,
	TEGRA_GPIO_BANK_ID_B,
	TEGRA_GPIO_BANK_ID_C,
	TEGRA_GPIO_BANK_ID_D,
	TEGRA_GPIO_BANK_ID_E,
	TEGRA_GPIO_BANK_ID_F,
	TEGRA_GPIO_BANK_ID_G,
	TEGRA_GPIO_BANK_ID_H,
	TEGRA_GPIO_BANK_ID_I,
	TEGRA_GPIO_BANK_ID_J,
	TEGRA_GPIO_BANK_ID_K,
	TEGRA_GPIO_BANK_ID_L,
	TEGRA_GPIO_BANK_ID_M,
	TEGRA_GPIO_BANK_ID_N,
	TEGRA_GPIO_BANK_ID_O,
	TEGRA_GPIO_BANK_ID_P,
	TEGRA_GPIO_BANK_ID_Q,
	TEGRA_GPIO_BANK_ID_R,
	TEGRA_GPIO_BANK_ID_T,
	TEGRA_GPIO_BANK_ID_X,
	TEGRA_GPIO_BANK_ID_Y,
	TEGRA_GPIO_BANK_ID_BB,
	TEGRA_GPIO_BANK_ID_CC,
};

/* GPIOs implemented by AON GPIO controller */
enum {
	TEGRA_GPIO_BANK_ID_S,
	TEGRA_GPIO_BANK_ID_U,
	TEGRA_GPIO_BANK_ID_V,
	TEGRA_GPIO_BANK_ID_W,
	TEGRA_GPIO_BANK_ID_Z,
	TEGRA_GPIO_BANK_ID_AA,
	TEGRA_GPIO_BANK_ID_EE,
	TEGRA_GPIO_BANK_ID_FF,
};

#define TEGRA_GPIO(bank, offset) \
	((TEGRA_GPIO_BANK_ID_##bank * 8) + offset)

#endif
