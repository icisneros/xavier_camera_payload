/*
 * Copyright (c) 2015-2018, NVIDIA CORPORATION.  All rights reserved.
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

#include <FreeRTOS.h>
#include <task.h>

#include <nvrm_drf.h>
#include <argpio_sw.h>

#include <gpio-client.h>
#include <gpio-provider.h>
#include <reg-access.h>
#include <tegra186-gpio.h>
#include <tegra-gpio-priv.h>
#include <irqs.h>
#include <macros.h>

enum tegra_gpio_irq_type {
	TEGRA_GPIO_IRQ_NONE,
	TEGRA_GPIO_IRQ_LEVEL,
	TEGRA_GPIO_IRQ_SINGLE_EDGE,
	TEGRA_GPIO_IRQ_DOUBLE_EDGE,
};

static uint32_t tegra_gpio_irq_types[] = {
	[GPIO_IRQ_NONE]		= TEGRA_GPIO_IRQ_NONE,
	[GPIO_IRQ_LEVEL]	= TEGRA_GPIO_IRQ_LEVEL,
	[GPIO_IRQ_SINGLE_EDGE]	= TEGRA_GPIO_IRQ_SINGLE_EDGE,
	[GPIO_IRQ_DOUBLE_EDGE]	= TEGRA_GPIO_IRQ_DOUBLE_EDGE,
};

enum tegra_gpio_irq_level {
	TEGRA_GPIO_IRQ_LOW_LEVEL = 0,
	TEGRA_GPIO_IRQ_FALLING_EDGE = 0,
	TEGRA_GPIO_IRQ_HIGH_LEVEL = 1,
	TEGRA_GPIO_IRQ_RISING_EDGE = 1,
};

static uint32_t tegra_gpio_irq_levels[] = {
	[GPIO_IRQ_LOW_LEVEL]	= TEGRA_GPIO_IRQ_LOW_LEVEL,
	[GPIO_IRQ_FALLING_EDGE]	= TEGRA_GPIO_IRQ_FALLING_EDGE,
	[GPIO_IRQ_HIGH_LEVEL]	= TEGRA_GPIO_IRQ_HIGH_LEVEL,
	[GPIO_IRQ_RISING_EDGE]	= TEGRA_GPIO_IRQ_RISING_EDGE,
};

static const inline bool gpio_valid(const struct tegra_gpio_id *id, int gpio)
{
	int bank = gpio / 8;

	return bank < id->bank_count;
}

static inline uint32_t reg(const struct tegra_gpio_id *id, int bank_gpio, uint32_t reg)
{
	int bank = bank_gpio / 8;
	int gpio = bank_gpio % 8;
	uint32_t bank_base = id->bank_bases[bank];

	return id->base_addr + bank_base + (reg - GPIO_N_ENABLE_CONFIG_00_0) +
		(gpio * (GPIO_N_ENABLE_CONFIG_01_0 - GPIO_N_ENABLE_CONFIG_00_0));
}

static int tegra186_gpio_enable_irq(void *idv, int gpio)
{
	uint32_t val;
	struct tegra_gpio_id *id = idv;

	if (!gpio_valid(id, gpio) || (id->irq == -1) ||
				(id->irq_handlers[gpio].handler == NULL))
		return -1;

	val = readl(reg(id, gpio, GPIO_N_ENABLE_CONFIG_00_0));
	val |= NV_DRF_DEF(GPIO, N_ENABLE_CONFIG_00, INTERRUPT_FUNCTION, ENABLE);
	writel(val, reg(id, gpio, GPIO_N_ENABLE_CONFIG_00_0));

	return 0;
}

static int tegra186_gpio_enable_timestamp(void *idv, int gpio)
{
	uint32_t val;
	struct tegra_gpio_id *id = idv;

	if (!gpio_valid(id, gpio))
		return -1;

	val = readl(reg(id, gpio, GPIO_N_ENABLE_CONFIG_00_0));
	val |= NV_DRF_DEF(GPIO, N_ENABLE_CONFIG_00, TIMESTAMPING_FUNCTION, ENABLE);
	writel(val, reg(id, gpio, GPIO_N_ENABLE_CONFIG_00_0));

	return 0;
}

static int tegra186_gpio_disable_timestamp(void *idv, int gpio)
{
	uint32_t val;
	struct tegra_gpio_id *id = idv;

	if (!gpio_valid(id, gpio))
		return -1;

	val = readl(reg(id, gpio, GPIO_N_ENABLE_CONFIG_00_0));
	val &= ~NV_DRF_DEF(GPIO, N_ENABLE_CONFIG_00, TIMESTAMPING_FUNCTION, ENABLE);
	writel(val, reg(id, gpio, GPIO_N_ENABLE_CONFIG_00_0));

	return 0;
}

static int tegra186_gpio_disable_irq(void *idv, int gpio)
{
	uint32_t val;
	struct tegra_gpio_id *id = idv;

	if (!gpio_valid(id, gpio) || (id->irq == -1) ||
				(id->irq_handlers[gpio].handler == NULL))
		return -1;

	val = readl(reg(id, gpio, GPIO_N_ENABLE_CONFIG_00_0));
	val &= ~NV_DRF_DEF(GPIO, N_ENABLE_CONFIG_00, INTERRUPT_FUNCTION, ENABLE);
	writel(val, reg(id, gpio, GPIO_N_ENABLE_CONFIG_00_0));

	return 0;
}

static int tegra186_gpio_set_irq_type(void *idv, int gpio, uint32_t type,
				uint32_t level)
{
	uint32_t val;
	const struct tegra_gpio_id *id = idv;

	if (!gpio_valid(id, gpio))
		return -1;

	if (!(level < ARRAY_SIZE(tegra_gpio_irq_levels) &&
			type < ARRAY_SIZE(tegra_gpio_irq_types)))
		return -1;

	val = readl(reg(id, gpio, GPIO_N_ENABLE_CONFIG_00_0));
	val = NV_FLD_SET_DRF_NUM(GPIO, N_ENABLE_CONFIG_00, TRIGGER_LEVEL,
				 tegra_gpio_irq_levels[level], val);
	val = NV_FLD_SET_DRF_NUM(GPIO, N_ENABLE_CONFIG_00, TRIGGER_TYPE,
				 tegra_gpio_irq_types[type], val);
	writel(val, reg(id, gpio, GPIO_N_ENABLE_CONFIG_00_0));

	return 0;
}

static void tegra186_gpio_clear_irq(const struct tegra_gpio_id *id, int gpio)
{
	uint32_t val;

	val = NV_DRF_DEF(GPIO, N_INTERRUPT_CLEAR_00, GPIO_INTERRUPT_CLEAR, CLEAR);
	writel(val, reg(id, gpio, GPIO_N_INTERRUPT_CLEAR_00_0));
}

static void tegra186_gpio_irq_handler(void *data)
{
	struct tegra_gpio_id *id = (struct tegra_gpio_id *)data;
	int bank;
	int gpio;
	uint32_t val;

	for (bank = 0; bank < id->bank_count; bank++) {
		val = readl(id->base_addr + id->bank_bases[bank] +
			    (id->irq_status_reg - GPIO_N_ENABLE_CONFIG_00_0));
		while (val) {
			gpio = __builtin_ctz(val);
			val &= ~BIT(gpio);
			gpio += bank * GPIOS_PER_BANK;
			tegra186_gpio_clear_irq(id, gpio);
			if (id->irq_handlers[gpio].handler != NULL)
				id->irq_handlers[gpio].handler(
						id->irq_handlers[gpio].data);
		}
	}
}

static int tegra186_gpio_set_irq_handler(void *idv, int gpio, void (*handler)(void *),
				void *data)
{
	struct tegra_gpio_id *id = idv;

	if (!gpio_valid(id, gpio) || (id->irq == -1))
		return -1;

	taskENTER_CRITICAL();
	id->irq_handlers[gpio].handler = handler;
	id->irq_handlers[gpio].data = data;
	taskEXIT_CRITICAL();

	return 0;
}

static int tegra186_gpio_clear_irq_handler(void *idv, int gpio)
{
	struct tegra_gpio_id *id = idv;
	uint32_t val;
	uint32_t irq_en;

	if (!gpio_valid(id, gpio) || (id->irq == -1))
		return -1;

	val = readl(reg(id, gpio, GPIO_N_ENABLE_CONFIG_00_0));
	irq_en = NV_DRF_VAL(GPIO, N_ENABLE_CONFIG_00, INTERRUPT_FUNCTION, val);
	if (irq_en)
		return -1;

	taskENTER_CRITICAL();
	id->irq_handlers[gpio].handler = NULL;
	id->irq_handlers[gpio].data = NULL;
	taskEXIT_CRITICAL();

	return 0;
}

static int tegra186_gpio_set_debounce(void *idv, int gpio, uint32_t debounce_ms)
{
	uint32_t val = 0;
	const struct tegra_gpio_id *id = idv;

	if (!gpio_valid(id, gpio))
		return -1;

	if (!debounce_ms) {
		val = readl(reg(id, gpio, GPIO_N_ENABLE_CONFIG_00_0));
		val &= ~NV_DRF_DEF(GPIO, N_ENABLE_CONFIG_00, DEBOUNCE_FUNCTION, ENABLE);
		writel(val, reg(id, gpio, GPIO_N_ENABLE_CONFIG_00_0));
		return 0;
	}

	val |= NV_DRF_NUM(GPIO, N_DEBOUNCE_THRESHOLD_00, DEBOUNCE_THRESHOLD, debounce_ms);
	writel(val, reg(id, gpio, GPIO_N_DEBOUNCE_THRESHOLD_00_0));
	val = readl(reg(id, gpio, GPIO_N_ENABLE_CONFIG_00_0));
	val |= NV_DRF_DEF(GPIO, N_ENABLE_CONFIG_00, DEBOUNCE_FUNCTION, ENABLE);
	writel(val, reg(id, gpio, GPIO_N_ENABLE_CONFIG_00_0));

	return 0;
}

static int tegra186_gpio_direction_in(void *idv, int gpio)
{
	uint32_t val;
	const struct tegra_gpio_id *id = idv;

	if (!gpio_valid(id, gpio))
		return -1;

	val = readl(reg(id, gpio, GPIO_N_ENABLE_CONFIG_00_0));
	val |=
		NV_DRF_DEF(GPIO, N_ENABLE_CONFIG_00, GPIO_ENABLE, ENABLE) |
		NV_DRF_DEF(GPIO, N_ENABLE_CONFIG_00, IN_OUT, IN);
	writel(val, reg(id, gpio, GPIO_N_ENABLE_CONFIG_00_0));

	return 0;
}

static int tegra186_gpio_direction_out(void *idv, int gpio, bool value)
{
	uint32_t val;
	const struct tegra_gpio_id *id = idv;

	if (!gpio_valid(id, gpio))
		return -1;

	val =
		NV_DRF_NUM(GPIO, N_OUTPUT_VALUE_00, GPIO_OUT_VAL, value);
	writel(val, reg(id, gpio, GPIO_N_OUTPUT_VALUE_00_0));

	val =
		NV_DRF_DEF(GPIO, N_OUTPUT_CONTROL_00, GPIO_OUT_CONTROL, DRIVEN);
	writel(val, reg(id, gpio, GPIO_N_OUTPUT_CONTROL_00_0));

	val = readl(reg(id, gpio, GPIO_N_ENABLE_CONFIG_00_0));
	val |=
		NV_DRF_DEF(GPIO, N_ENABLE_CONFIG_00, GPIO_ENABLE, ENABLE) |
		NV_DRF_DEF(GPIO, N_ENABLE_CONFIG_00, IN_OUT, OUT);
	writel(val, reg(id, gpio, GPIO_N_ENABLE_CONFIG_00_0));

	return 0;
}

static int tegra186_gpio_get_output_value(void *idv, int gpio)
{
	uint32_t val;
	uint32_t gpio_cfg;
	const struct tegra_gpio_id *id = idv;

	if (!gpio_valid(id, gpio))
		return -1;

	val = readl(reg(id, gpio, GPIO_N_ENABLE_CONFIG_00_0));
	gpio_cfg = NV_DRF_VAL(GPIO, N_ENABLE_CONFIG_00, IN_OUT, val);
	if (!gpio_cfg)
		return -1;

	val = readl(reg(id, gpio, GPIO_N_OUTPUT_VALUE_00_0));

	return !!NV_DRF_VAL(GPIO, N_OUTPUT_VALUE_00, GPIO_OUT_VAL, val);
}

static int tegra186_gpio_get_input_value(void *idv, int gpio)
{
	uint32_t val;
	const struct tegra_gpio_id *id = idv;

	if (!gpio_valid(id, gpio))
		return -1;

	val = readl(reg(id, gpio, GPIO_N_INPUT_00_0));

	return !!NV_DRF_VAL(GPIO, N_INPUT_00, GPIO_IN, val);
}

static int tegra186_gpio_get_value(void *idv, int gpio)
{
	uint32_t val;
	uint32_t gpio_cfg;
	const struct tegra_gpio_id *id = idv;

	if (!gpio_valid(id, gpio))
		return -1;

	val = readl(reg(id, gpio, GPIO_N_ENABLE_CONFIG_00_0));
	gpio_cfg = NV_DRF_VAL(GPIO, N_ENABLE_CONFIG_00, IN_OUT, val);
	if (!gpio_cfg) {
		val = readl(reg(id, gpio, GPIO_N_INPUT_00_0));
		return !!NV_DRF_VAL(GPIO, N_INPUT_00, GPIO_IN, val);
	} else {
		val = readl(reg(id, gpio, GPIO_N_OUTPUT_VALUE_00_0));
		return !!NV_DRF_VAL(GPIO, N_OUTPUT_VALUE_00, GPIO_OUT_VAL, val);
	}
}

static int tegra186_gpio_set_value(void *idv, int gpio, bool value)
{
	uint32_t val;
	const struct tegra_gpio_id *id = idv;

	if (!gpio_valid(id, gpio))
		return -1;

	val =
		NV_DRF_NUM(GPIO, N_OUTPUT_VALUE_00, GPIO_OUT_VAL, value);
	writel(val, reg(id, gpio, GPIO_N_OUTPUT_VALUE_00_0));

	return 0;
}

static int tegra186_gpio_suspend(void *idv)
{
	struct tegra_gpio_id *id = idv;
	int i;

	if (id->irq == -1)
		return 0;

	if (id->irqs == NULL) {
		irq_disable(id->irq);
	} else {
		for (i = 0; i < id->nirqs; i++)
			irq_disable(id->irqs[i]);
	}

	return 0;
}

static int tegra186_gpio_resume(void *idv)
{
	struct tegra_gpio_id *id = idv;
	int i;

	if (id->irq == -1)
		return 0;

	if (id->irqs == NULL) {
		irq_enable(id->irq);
	} else {
		for (i = 0; i < id->nirqs; i++)
			irq_enable(id->irqs[i]);
	}

	return 0;
}
static int tegra186_gpio_init(void *idv)
{
	struct tegra_gpio_id *id = idv;
	int i;

	if (id->irq == -1)
		return 0;

	if (id->irqs == NULL) {
		irq_set_handler(id->irq, tegra186_gpio_irq_handler, id);
		irq_enable(id->irq);
	} else {
		for (i = 0; i < id->nirqs; i++) {
			irq_set_handler(id->irqs[i], tegra186_gpio_irq_handler,
					id);
			irq_enable(id->irqs[i]);
		}
	}

	return 0;
}

const struct gpio_ops tegra186_gpio_ops = {
	.init = tegra186_gpio_init,
	.suspend = tegra186_gpio_suspend,
	.resume = tegra186_gpio_resume,
	.direction_in = tegra186_gpio_direction_in,
	.direction_out = tegra186_gpio_direction_out,
	.get_value = tegra186_gpio_get_value,
	.get_output_value = tegra186_gpio_get_output_value,
	.get_input_value = tegra186_gpio_get_input_value,
	.set_value = tegra186_gpio_set_value,
	.set_debounce = tegra186_gpio_set_debounce,
	.set_irq_type = tegra186_gpio_set_irq_type,
	.enable_irq = tegra186_gpio_enable_irq,
	.enable_timestamp = tegra186_gpio_enable_timestamp,
	.disable_timestamp = tegra186_gpio_disable_timestamp,
	.disable_irq = tegra186_gpio_disable_irq,
	.set_irq_handler = tegra186_gpio_set_irq_handler,
	.clear_irq_handler = tegra186_gpio_clear_irq_handler,
};
