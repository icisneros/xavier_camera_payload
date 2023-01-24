/*
 * Copyright (c) 2015-2016, NVIDIA CORPORATION.  All rights reserved.
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

#include <macros.h>
#include <gpio-client.h>
#include <gpio-provider.h>

static struct {
	const struct gpio_ops *ops;
	void *priv;
} chips[4];

int gpio_chip_register(int chip_id, const struct gpio_ops *ops, void *priv)
{
	int ret;

	if (chip_id >= ARRAY_SIZE(chips))
		return -1;

	if (chips[chip_id].ops)
		return -1;

	chips[chip_id].ops = ops;
	chips[chip_id].priv = priv;

	ret = ops->init(priv);
	if (ret) {
		chips[chip_id].ops = NULL;
		chips[chip_id].priv = NULL;
	}

	return ret;
}

int gpio_direction_in(int gpio_global)
{
	int chip_id = gpio_chip_id_of_global_id(gpio_global);
	int gpio_id = gpio_gpio_id_of_global_id(gpio_global);
	const struct gpio_ops *ops;
	void *priv;

	if (chip_id >= ARRAY_SIZE(chips))
		return -1;

	ops = chips[chip_id].ops;
	priv = chips[chip_id].priv;
	if (!ops)
		return -1;

	return ops->direction_in(priv, gpio_id);
}

int gpio_direction_out(int gpio_global, bool value)
{
	int chip_id = gpio_chip_id_of_global_id(gpio_global);
	int gpio_id = gpio_gpio_id_of_global_id(gpio_global);
	const struct gpio_ops *ops;
	void *priv;

	if (chip_id >= ARRAY_SIZE(chips))
		return -1;

	ops = chips[chip_id].ops;
	priv = chips[chip_id].priv;
	if (!ops)
		return -1;

	return ops->direction_out(priv, gpio_id, value);
}

int gpio_get_output_value(int gpio_global)
{
	int chip_id = gpio_chip_id_of_global_id(gpio_global);
	int gpio_id = gpio_gpio_id_of_global_id(gpio_global);
	const struct gpio_ops *ops;
	void *priv;

	if (chip_id >= ARRAY_SIZE(chips))
		return -1;

	ops = chips[chip_id].ops;
	priv = chips[chip_id].priv;
	if (!ops)
		return -1;

	return ops->get_output_value(priv, gpio_id);
}

int gpio_get_input_value(int gpio_global)
{
	int chip_id = gpio_chip_id_of_global_id(gpio_global);
	int gpio_id = gpio_gpio_id_of_global_id(gpio_global);
	const struct gpio_ops *ops;
	void *priv;

	if (chip_id >= ARRAY_SIZE(chips))
		return -1;

	ops = chips[chip_id].ops;
	priv = chips[chip_id].priv;
	if (!ops)
		return -1;

	return ops->get_input_value(priv, gpio_id);
}

int gpio_get_value(int gpio_global)
{
	int chip_id = gpio_chip_id_of_global_id(gpio_global);
	int gpio_id = gpio_gpio_id_of_global_id(gpio_global);
	const struct gpio_ops *ops;
	void *priv;

	if (chip_id >= ARRAY_SIZE(chips))
		return -1;

	ops = chips[chip_id].ops;
	priv = chips[chip_id].priv;
	if (!ops)
		return -1;

	return ops->get_value(priv, gpio_id);
}

int gpio_set_value(int gpio_global, bool value)
{
	int chip_id = gpio_chip_id_of_global_id(gpio_global);
	int gpio_id = gpio_gpio_id_of_global_id(gpio_global);
	const struct gpio_ops *ops;
	void *priv;

	if (chip_id >= ARRAY_SIZE(chips))
		return -1;

	ops = chips[chip_id].ops;
	priv = chips[chip_id].priv;
	if (!ops)
		return -1;

	return ops->set_value(priv, gpio_id, value);
}

int gpio_set_debounce(int gpio_global, uint32_t debounce_ms)
{
	int chip_id = gpio_chip_id_of_global_id(gpio_global);
	int gpio_id = gpio_gpio_id_of_global_id(gpio_global);
	const struct gpio_ops *ops;
	void *priv;

	if (chip_id >= ARRAY_SIZE(chips))
		return -1;

	ops = chips[chip_id].ops;
	priv = chips[chip_id].priv;
	if (!ops)
		return -1;

	return ops->set_debounce(priv, gpio_id, debounce_ms);
}

int gpio_set_irq_type(int gpio_global, enum gpio_irq_type type,
			enum gpio_irq_level level)
{
	int chip_id = gpio_chip_id_of_global_id(gpio_global);
	int gpio_id = gpio_gpio_id_of_global_id(gpio_global);
	const struct gpio_ops *ops;
	void *priv;

	if (chip_id >= ARRAY_SIZE(chips))
		return -1;

	ops = chips[chip_id].ops;
	priv = chips[chip_id].priv;
	if (!ops)
		return -1;

	return ops->set_irq_type(priv, gpio_id, type, level);
}

int gpio_enable_irq(int gpio_global)
{
	int chip_id = gpio_chip_id_of_global_id(gpio_global);
	int gpio_id = gpio_gpio_id_of_global_id(gpio_global);
	const struct gpio_ops *ops;
	void *priv;

	if (chip_id >= ARRAY_SIZE(chips))
		return -1;

	ops = chips[chip_id].ops;
	priv = chips[chip_id].priv;
	if (!ops)
		return -1;

	return ops->enable_irq(priv, gpio_id);
}

int gpio_enable_timestamp(int gpio_global)
{
	int chip_id = gpio_chip_id_of_global_id(gpio_global);
	int gpio_id = gpio_gpio_id_of_global_id(gpio_global);
	const struct gpio_ops *ops;
	void *priv;

	if (chip_id >= ARRAY_SIZE(chips))
		return -1;

	ops = chips[chip_id].ops;
	priv = chips[chip_id].priv;
	if (!ops)
		return -1;

	return ops->enable_timestamp(priv, gpio_id);
}

int gpio_disable_timestamp(int gpio_global)
{
	int chip_id = gpio_chip_id_of_global_id(gpio_global);
	int gpio_id = gpio_gpio_id_of_global_id(gpio_global);
	const struct gpio_ops *ops;
	void *priv;

	if (chip_id >= ARRAY_SIZE(chips))
		return -1;

	ops = chips[chip_id].ops;
	priv = chips[chip_id].priv;
	if (!ops)
		return -1;

	return ops->disable_timestamp(priv, gpio_id);
}

int gpio_disable_irq(int gpio_global)
{
	int chip_id = gpio_chip_id_of_global_id(gpio_global);
	int gpio_id = gpio_gpio_id_of_global_id(gpio_global);
	const struct gpio_ops *ops;
	void *priv;

	if (chip_id >= ARRAY_SIZE(chips))
		return -1;

	ops = chips[chip_id].ops;
	priv = chips[chip_id].priv;
	if (!ops)
		return -1;

	return ops->disable_irq(priv, gpio_id);
}

int gpio_set_irq_handler(int gpio_global, void (*handler)(void *), void *data)
{
	int chip_id = gpio_chip_id_of_global_id(gpio_global);
	int gpio_id = gpio_gpio_id_of_global_id(gpio_global);
	const struct gpio_ops *ops;
	void *priv;

	if (chip_id >= ARRAY_SIZE(chips))
		return -1;

	ops = chips[chip_id].ops;
	priv = chips[chip_id].priv;
	if (!ops)
		return -1;

	return ops->set_irq_handler(priv, gpio_id, handler, data);
}

int gpio_clear_irq_handler(int gpio_global)
{
	int chip_id = gpio_chip_id_of_global_id(gpio_global);
	int gpio_id = gpio_gpio_id_of_global_id(gpio_global);
	const struct gpio_ops *ops;
	void *priv;

	if (chip_id >= ARRAY_SIZE(chips))
		return -1;

	ops = chips[chip_id].ops;
	priv = chips[chip_id].priv;
	if (!ops)
		return -1;

	return ops->clear_irq_handler(priv, gpio_id);
}
