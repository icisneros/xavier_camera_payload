/*
 * Copyright (c) 2016-2017, NVIDIA CORPORATION. All rights reserved.
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

#include <address_map_new.h>
#include <argpio_sw.h>
#include <argpio_aon_sw.h>

#include <macros.h>
#include <tegra-gpio-priv.h>
#include <gpio-tegra-hw-params.h>

static const uint32_t tegra194_gpio_main_bases[] = {
	GPIO_A_ENABLE_CONFIG_00_0,
	GPIO_B_ENABLE_CONFIG_00_0,
	GPIO_C_ENABLE_CONFIG_00_0,
	GPIO_D_ENABLE_CONFIG_00_0,
	GPIO_E_ENABLE_CONFIG_00_0,
	GPIO_F_ENABLE_CONFIG_00_0,
	GPIO_G_ENABLE_CONFIG_00_0,
	GPIO_H_ENABLE_CONFIG_00_0,
	GPIO_I_ENABLE_CONFIG_00_0,
	GPIO_J_ENABLE_CONFIG_00_0,
	GPIO_K_ENABLE_CONFIG_00_0,
	GPIO_L_ENABLE_CONFIG_00_0,
	GPIO_M_ENABLE_CONFIG_00_0,
	GPIO_N_ENABLE_CONFIG_00_0,
	GPIO_O_ENABLE_CONFIG_00_0,
	GPIO_P_ENABLE_CONFIG_00_0,
	GPIO_Q_ENABLE_CONFIG_00_0,
	GPIO_R_ENABLE_CONFIG_00_0,
	GPIO_S_ENABLE_CONFIG_00_0,
	GPIO_T_ENABLE_CONFIG_00_0,
	GPIO_U_ENABLE_CONFIG_00_0,
	GPIO_V_ENABLE_CONFIG_00_0,
	GPIO_W_ENABLE_CONFIG_00_0,
	GPIO_X_ENABLE_CONFIG_00_0,
	GPIO_Y_ENABLE_CONFIG_00_0,
	GPIO_Z_ENABLE_CONFIG_00_0,
	GPIO_FF_ENABLE_CONFIG_00_0,
	GPIO_GG_ENABLE_CONFIG_00_0,
};

static struct gpio_irq_handler tegra194_gpio_main_handlers[
				ARRAY_SIZE(tegra194_gpio_main_bases) *
				GPIOS_PER_BANK];

static uint32_t tegra194_main_gpio_id_irqs[] = {
	TEGRA_MAIN_GPIO_ID0_IRQ,
	TEGRA_MAIN_GPIO_ID1_IRQ,
	TEGRA_MAIN_GPIO_ID2_IRQ,
	TEGRA_MAIN_GPIO_ID3_IRQ,
	TEGRA_MAIN_GPIO_ID4_IRQ,
	TEGRA_MAIN_GPIO_ID5_IRQ,
};

struct tegra_gpio_id tegra194_gpio_id_main = {
	.devname = "gpio-main",
	.base_addr = NV_ADDRESS_MAP_GPIO_CTL0_BASE,
	.bank_count = ARRAY_SIZE(tegra194_gpio_main_bases),
	.bank_bases = tegra194_gpio_main_bases,
	.irq = TEGRA_MAIN_GPIO_IRQ,
	.irqs = tegra194_main_gpio_id_irqs,
	.nirqs = ARRAY_SIZE(tegra194_main_gpio_id_irqs),
	.irq_handlers = tegra194_gpio_main_handlers,
	.irq_status_reg = TEGRA_GPIO_IRQ_STATUS_REG,
};

static const uint32_t tegra194_gpio_aon_bases[] = {
	GPIO_AA_ENABLE_CONFIG_00_0,
	GPIO_BB_ENABLE_CONFIG_00_0,
	GPIO_CC_ENABLE_CONFIG_00_0,
	GPIO_DD_ENABLE_CONFIG_00_0,
	GPIO_EE_ENABLE_CONFIG_00_0,
};

static struct gpio_irq_handler tegra194_gpio_aon_handlers[
				ARRAY_SIZE(tegra194_gpio_aon_bases) *
				GPIOS_PER_BANK];

struct tegra_gpio_id tegra194_gpio_id_aon = {
	.devname = "gpio-aon",
	.base_addr = NV_ADDRESS_MAP_AON_GPIO_0_BASE,
	.bank_count = ARRAY_SIZE(tegra194_gpio_aon_bases),
	.bank_bases = tegra194_gpio_aon_bases,
	.irq = TEGRA_AON_GPIO_IRQ,
	.irq_handlers = tegra194_gpio_aon_handlers,
	.irq_status_reg = TEGRA_GPIO_IRQ_STATUS_REG,
};
