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

#ifndef _GPIO_PROVIDER_H_
#define _GPIO_PROVIDER_H_

/*
 * The GPIO API provides a single set of functions that dispatch GPIO-related
 * requests from client code to a set of registered GPIO providers. This
 * allows code that uses GPIOs to be unaware which hardware module actually
 * implements those GPIOs. This is useful since boards often use different
 * GPIOs for the same function; for example an SPI driver might need to
 * manipulate a chip-select, or a camera flash driver might use a GPIO to turn
 * the flash LED on. In neither case should the client care whether the GPIO
 * is physically controlled by Tegra's main GPIO controller, AON GPIO
 * controller, or even some I2C-based GPIO expander.
 *
 * This file contains definitions that:
 * - GPIO controllers will use to provide services to GPIO clients.
 * - Board-level code will use to register GPIO providers with the GPIO API.
 *
 * See <gpio-client.h> for the GPIO client API.
 *
 * Each GPIO provider implements N GPIOs, as dictate by HW. All callbacks from
 * the GPIO API into a provider function use a chip-relative 0-based GPIO ID.
 * GPIO clients use a system-global GPIO ID. Each (instance of a) GPIO
 * provider is assigned a unique "chip ID". The chip ID and provider-
 * relative GPIO ID are merged together to form a global GPIO ID. All GPIO
 * clients use global GPIO IDs. Board-level code is expected to assign GPIO
 * chip IDs, and construct global GPIO IDs to pass to GPIO clients during
 * initialization, or even via statically constructed values at compile-time.
 */

#include <stdbool.h>
#include <stdint.h>

/*
 * GPIO provider "callback"/implementation functions/operations.
 *
 * These have the same semantics as the functions in gpio-client.h. The
 * differences are:
 * - They take a "priv" pointer that the GPIO provider functions can use to
 *   store/locate data they require for their operation. An example might be
 *   a hardware module base address.
 * - The gpio parameter is chip-relative rather than global.
 */
typedef int (*gpio_init_func)(void *priv);
typedef int (*gpio_suspend_func)(void *priv);
typedef int (*gpio_resume_func)(void *priv);
typedef int (*gpio_direction_in_func)(void *priv, int gpio);
typedef int (*gpio_direction_out_func)(void *priv, int gpio, bool value);
typedef int (*gpio_get_value_func)(void *priv, int gpio);
typedef int (*gpio_get_output_value_func)(void *priv, int gpio);
typedef int (*gpio_get_input_value_func)(void *priv, int gpio);
typedef int (*gpio_set_value_func)(void *priv, int gpio, bool value);
typedef int (*gpio_set_debounce_func)(void *priv, int gpio, uint32_t debounce_ms);
typedef int (*gpio_set_irq_type_func)(void *priv, int gpio, uint32_t type,
					uint32_t level);
typedef int (*gpio_enable_irq_func)(void *priv, int gpio);
typedef int (*gpio_enable_timestamp_func)(void *priv, int gpio);
typedef int (*gpio_disable_timestamp_func)(void *priv, int gpio);
typedef int (*gpio_disable_irq_func)(void *priv, int gpio);
typedef int (*gpio_set_irq_handler_func)(void *priv, int gpio,
					void (*fn)(void *), void *data);
typedef int (*gpio_clear_irq_handler_func)(void *priv, int gpio);


struct gpio_ops {
	gpio_init_func init;
	gpio_suspend_func suspend;
	gpio_resume_func resume;
	gpio_direction_in_func direction_in;
	gpio_direction_out_func direction_out;
	gpio_get_value_func get_value;
	gpio_get_output_value_func get_output_value;
	gpio_get_input_value_func get_input_value;
	gpio_set_value_func set_value;
	gpio_set_debounce_func set_debounce;
	gpio_set_irq_type_func set_irq_type;
	gpio_enable_irq_func enable_irq;
	gpio_enable_timestamp_func enable_timestamp;
	gpio_disable_timestamp_func disable_timestamp;
	gpio_disable_irq_func disable_irq;
	gpio_set_irq_handler_func set_irq_handler;
	gpio_clear_irq_handler_func clear_irq_handler;
};

/*
 * Register a GPIO provider with the GPIO API.
 *
 * Parameters:
 * chip_id:	The unique ID of the GPIO provider. This is used to generate
 * 		global GPIO IDs.
 * ops:		The set of callback/operation functions the GPIO API should
 * 		to handler requests for this provider's GPIOS.
 * priv:	A piece of data that is passed to each of the provider's
 * 		functions. This can provide configuration data to the
 * 		provider, such as HW module base address.
 *
 * Returns:
 * 0:		Success
 * Negative:	Error
 */
int gpio_chip_register(int chip_id, const struct gpio_ops *ops, void *priv);

/*
 * Construct a global GPIO ID.
 *
 * No error-range-checking is performed on the parameters or returned value.
 *
 * Parameters:
 * chip_id:	The unique ID of the GPIO provider.
 * gpio:	The provider-relative ID of the GPIO.
 *
 * Returns:
 * The global GPIO ID.
 */
static inline const int gpio_global_id(int chip_id, int gpio)
{
	return (chip_id << 16) + gpio;
}

/*
 * Extract a GPIO chip ID from a global GPIO ID.
 *
 * No error-range-checking is performed on the parameters or returned value.
 *
 * Parameters:
 * global_id:	The global GPIO ID of the GPIO.
 *
 * Returns:
 * The chip ID of the provider of the GPIO.
 */
static inline const int gpio_chip_id_of_global_id(int global_id)
{
	return global_id >> 16;
}

/*
 * Extract a chip-relative GPIO ID from a global GPIO ID.
 *
 * No error-range-checking is performed on the parameters or returned value.
 *
 * Parameters:
 * global_id:	The global GPIO ID of the GPIO.
 *
 * Returns:
 * The chip-relative GPIO ID of the GPIO.
 */
static inline const int gpio_gpio_id_of_global_id(int global_id)
{
	return global_id & 0xffff;
}

#endif
