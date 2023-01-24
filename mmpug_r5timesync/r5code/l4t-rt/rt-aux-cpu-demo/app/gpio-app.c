/*
 * Copyright (c) 2017-2019, NVIDIA CORPORATION.  All rights reserved.
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

/* Note: Follow doc/gpio.txt to successfully run this demo app */

#include <FreeRTOS.h>

#include <err-hook.h>
#include <irqs.h>
#include <irqs-hw.h>
#include <macros.h>
#include <printf-isr.h>
#include <reg-access.h>
#include <stdio.h>
#include <task.h>

#include <gpio-provider.h>
#include <gpio-client.h>

/* gpio-aon.h has GPIO_APP* defines */
#include "gpio-aon.h"
#include "gpio-app.h"

/*#define GPIO_OUT_SET_DELAY	3000

void can_gpio_irq_handler(void *data);

void can_gpio_irq_handler(void *data)
{
	printf_isr("%s - gpio irq triggered - setting GPIO_APP_OUT to 0 \r\n",
		   __func__);
	gpio_set_value(GPIO_APP_OUT, 0);
}*/

/*static portTASK_FUNCTION(gpio_app_task, pvParameters)
{
	(void)pvParameters;
	int val;

	val = gpio_direction_out(GPIO_APP_OUT, 0);
	if (val) {
		error_hook("gpio_direction_out failed\r\n");
		return;
	}
	val = gpio_direction_in(GPIO_APP_IN);
	if (val) {
		error_hook("gpio_direction_in failed\r\n");
		return;
	}

	val = gpio_set_irq_type(GPIO_APP_IN, GPIO_IRQ_SINGLE_EDGE,
				GPIO_IRQ_RISING_EDGE);
	if (val) {
		error_hook("gpio_set_irq_type failed\r\n");
		return;
	}

	val = gpio_set_irq_handler(GPIO_APP_IN, can_gpio_irq_handler, NULL);
	if (val) {
		error_hook("gpio_set_irq_handler failed\r\n");
		return;
	}

	val = gpio_enable_irq(GPIO_APP_IN);
	if (val) {
		error_hook("gpio_enable_irq failed\r\n");
		return;
	}

	while (1) {
		printf("%s - Setting GPIO_APP_OUT to 1 - IRQ should trigger\r\n",
		       __func__);
		gpio_set_value(GPIO_APP_OUT, 1);
		vTaskDelay(GPIO_OUT_SET_DELAY);
		gpio_set_value(GPIO_APP_OUT, 0);
		vTaskDelay(GPIO_OUT_SET_DELAY);
	};
}*/

void gpio_app_init(void)
{
/*	int val = xTaskCreate(gpio_app_task, "gpioapp", 512, NULL,
			      tskIDLE_PRIORITY, NULL);
	if (val != pdPASS)
		error_hook("xTaskCreate for gpio_app_task failed\r\n");*/
}
