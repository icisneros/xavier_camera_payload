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

#ifndef _GPIO_CLIENT_H_
#define _GPIO_CLIENT_H_

#include <stdbool.h>
#include <stdint.h>

enum gpio_irq_type {
	GPIO_IRQ_NONE,
	GPIO_IRQ_LEVEL,
	GPIO_IRQ_SINGLE_EDGE,
	GPIO_IRQ_DOUBLE_EDGE,
};

enum gpio_irq_level {
	GPIO_IRQ_LOW_LEVEL,
	GPIO_IRQ_FALLING_EDGE,
	GPIO_IRQ_HIGH_LEVEL,
	GPIO_IRQ_RISING_EDGE,
};

/*
 * Configure a GPIO to be an input.
 *
 * Parameters:
 * gpio:	The global GPIO ID to configure.
 *
 * Returns:
 * 0:		Success
 * Negative:	Error
 */
int gpio_direction_in(int gpio);

/*
 * Configure a GPIO to be an output, and set its initial output value.
 *
 * Parameters:
 * gpio:	The global GPIO ID to configure.
 * value:	The value to drive on the GPIO.
 * 		This is the raw value at the pin; the GPIO API performs no
 * 		internal conversions to account for active-low signals, etc.
 *
 * Returns:
 * 0:		Success
 * Negative:	Error
 */
int gpio_direction_out(int gpio, bool value);

/*
 * Retrieve the value of a GPIO.
 *
 * This function's behaviour is undefined if called on a GPIO that was not
 * previously configured.
 *
 * The raw value at the pin is returned; the GPIO API performs no internal
 * conversions to account for active-low signals, etc.
 *
 * Parameters:
 * gpio:	The global GPIO ID to read.
 *
 * Returns:
 * 0:		Success, signal is low.
 * 1:		Success, signal is high.
 * Negative:	Error
 */
int gpio_get_value(int gpio);

/*
 * Retrieve the value of an output GPIO.
 *
 * This function's behaviour is undefined if called on a GPIO that was not
 * previously configured as an output.
 *
 * The raw value at the pin is returned; the GPIO API performs no internal
 * conversions to account for active-low signals, etc.
 *
 * Parameters:
 * gpio:	The global GPIO ID to read.
 *
 * Returns:
 * 0:		Success, output signal is low.
 * 1:		Success, output signal is high.
 * Negative:	Error
 */
int gpio_get_output_value(int gpio);

/*
 * Retrieve the value of an input GPIO.
 *
 * This function's behaviour is undefined if called on a GPIO that was not
 * previously configured as an input.
 *
 * The raw value at the pin is returned; the GPIO API performs no internal
 * conversions to account for active-low signals, etc.
 *
 * Parameters:
 * gpio:	The global GPIO ID to read.
 *
 * Returns:
 * 0:		Success, input signal is low.
 * 1:		Success, input signal is high.
 * Negative:	Error
 */
int gpio_get_input_value(int gpio);

/*
 * Set the output value for a GPIO.
 *
 * This function's behaviour is undefined if called on a GPIO that was not
 * previously configured as an output.
 *
 * Parameters:
 * gpio:	The global GPIO ID to configure.
 * value:	The value to drive on the GPIO.
 * 		This is the raw value at the pin; the GPIO API performs no
 * 		internal conversions to account for active-low signals, etc.
 *
 * Returns:
 * 0:		Success
 * Negative:	Error
 */
int gpio_set_value(int gpio, bool value);

/*
 * Set the debounce value for a GPIO.
 *
 * This function's behaviour is undefined if called on a GPIO that was not
 * previously configured as an output.
 *
 * Parameters:
 * gpio:	The global GPIO ID to configure.
 * debounce_ms:	The value to program for debounce in miliseconds.
 *
 * Returns:
 * 0:		Success
 * Negative:	Error
 */
int gpio_set_debounce(int gpio, uint32_t debounce_ms);

/*
 * Set the irq type for a GPIO.
 *
 * This function's behaviour is undefined if called on a GPIO that was not
 * previously configured as an input.
 *
 * Paramters:
 * gpio:	The global GPIO ID to configure.
 * type:	Indicates the type based on enum gpio_irq_type.
 * level:	Indicates the level based on enum gpio_irq_level.
 *
 * Returns:
 * 0:		Success
 * Negative:	Error
 */
int gpio_set_irq_type(int gpio, enum gpio_irq_type type,
			enum gpio_irq_level level);

/*
 * Enable the irq for a GPIO.
 *
 * This function's behaviour is undefined if called on a GPIO that was not
 * previously configured as an input.
 *
 * Paramters:
 * gpio:	The global GPIO ID to configure.
 *
 * Returns:
 * 0:		Success
 * Negative:	Error
 */
int gpio_enable_irq(int gpio);

/*
 * Enable the timestamping functionality for a GPIO.
 *
 * This function's behaviour is undefined if called on a GPIO that was not
 * previously configured as an input. This API needs to be called for a GPIO
 * that you are interested in monitoring using the HW timestamping engines
 * such as GTE.
 *
 * Paramters:
 * gpio:	The global GPIO ID to configure.
 *
 * Returns:
 * 0:		Success
 * Negative:	Error
 */
int gpio_enable_timestamp(int gpio);

/*
 * Disable the timestamping functionality for a GPIO.
 *
 * This function's behaviour is undefined if called on a GPIO that was not
 * previously configured as an input.
 *
 * Paramters:
 * gpio:	The global GPIO ID to configure.
 *
 * Returns:
 * 0:		Success
 * Negative:	Error
 */
int gpio_disable_timestamp(int gpio);

/*
 * Disable the irq for a GPIO.
 *
 * This function's behaviour is undefined if called on a GPIO that was not
 * previously configured as an input.
 *
 * Paramters:
 * gpio:	The global GPIO ID to configure.
 *
 * Returns:
 * 0:		Success
 * Negative:	Error
 */
int gpio_disable_irq(int gpio);

/*
 * Register an interrupt service routine for a GPIO.
 *
 * This function's behaviour is undefined if called on a GPIO that was not
 * previously configured as an input.
 *
 * Parameters:
 * gpio:	The global GPIO ID to configure.
 * func:	service routine.
 * data:	opaque data pointer for the service routine.
 *
 * Returns:
 * 0:		Success
 * Negative:	Error
 */
int gpio_set_irq_handler(int gpio, void (*func)(void *), void *data);

/*
 * Unregister an interrupt service routine for a GPIO.
 *
 * This function's behaviour is undefined if called on a GPIO that was not
 * previously configured as an input.
 *
 * Parameters:
 * gpio:	The global GPIO ID to configure.
 *
 * Returns:
 * 0:		Success
 * Negative:	Error
 */
int gpio_clear_irq_handler(int gpio);

#endif
