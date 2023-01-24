/*
 * Copyright (c) 2014-2020, NVIDIA CORPORATION. All rights reserved.
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
#include <semphr.h>

#include <timeserver.h>

#include <address_map_new.h>
#include <arm-vic.h>
#include <arclk_rst.h>
#include <araopm.h>
#include <clk-tegra.h>
#include <cache.h>
#include <delay.h>
#include <err-hook.h>
#include <irqs.h>
#include <irqs-hw.h>
#include <macros.h>
#include <nvrm_drf.h>
#include <reg-access.h>
#include <spe-pm.h>
#include <spe-vic.h>
#include <stdio.h>
#include <task.h>
#include <timers.h>
#include <tke-tegra.h>
#include <tke-tegra-hw.h>

#include <bpmp-ipc.h>
#include <hsp-tegra.h>
#include <hsp-tegra-hw.h>
#include <ivc-channels.h>

#if defined(ENABLE_GPCDMA_FUNC)
#include <tegra-gpcdma.h>
#include <tegra-gpcdma-hw.h>
#endif

#if defined(ENABLE_GPIO_APP)
#include <gpio-provider.h>
#include <gpio-aon.h>
#include "app/gpio-app.h"
#endif

#if defined(ENABLE_TIMER_APP)
#include "app/timer-app.h"
#endif

#if defined(ENABLE_I2C_APP)
#include "app/i2c-app.h"
#endif

#if defined(ENABLE_UART_APP)
#include "app/uart-app.h"
#endif

#if defined(ENABLE_SPI_APP)
#include "app/spi-app.h"
#endif

#if defined(ENABLE_GTE_APP)
#include "app/gte-app.h"
#endif

#if defined(ENABLE_CAN_APP)
#include "app/can-app.h"
#endif

#if defined(ENABLE_AODMIC_APP)
#include "app/aodmic-app.h"
#endif

#include "config.h"
#include "init_padctrl.h"
#include "debug_init.h"

#define ACTIVE_MODE_IDX 0
#define ACTIVE_IRQFIQ_MODE_IDX 1

void vApplicationStackOverflowHook(TimerHandle_t pxTask, signed char *pcTaskName);

#define WAIT_TIMEOUT_MS 3000
#define MB1_DB_IRQ (NV_AON_INTERRUPT_VIC1_BASE + NV_AON_INTERRUPT_TOP0_HSP_DB)

static SemaphoreHandle_t late_init_sem;



struct TimeServerInformation timeserverdata;



static void late_init(void)
{
	debug_init();
	if (ivc_init_channels_ccplex())
		printf("ivc_init_channels_ccplex() failed\r\n");
}

static void late_init_db_irq(void *data)
{
	BaseType_t yield;
	(void)data;

	if (xSemaphoreGiveFromISR(late_init_sem, &yield) != pdTRUE)
		error_hook("Couldn't give semaphore.");

	irq_disable(MB1_DB_IRQ);
	irq_set_handler(MB1_DB_IRQ, NULL, NULL);
	portYIELD_FROM_ISR(yield);
}

static portTASK_FUNCTION(late_init_task, params)
{
	(void)params;
	if (xSemaphoreTake(late_init_sem, WAIT_TIMEOUT_MS / portTICK_PERIOD_MS)
			== pdFALSE)
		warning_hook("No doorbell received from MB1");
	late_init();
	vSemaphoreDelete(late_init_sem);
	late_init_sem = NULL;
	vTaskDelete(NULL);
}

int main(void)
{
	//Initialize values for Time Server
	timeserverdata.ticks_persecond = 31250000;
	timeserverdata.phaseoffset_ticks = 0;
	//For now set synced to true to test pulses. Later once everything is working set it to false.
	timeserverdata.synced=0;
	
	spe_vic_init();

	/* select FIQ interrupt */
	arm_vic_write_intselect(NV_ADDRESS_MAP_AON_VIC_0_BASE, BIT(NV_AON_INTERRUPT_WDTFIQ));

	init_padctrl();

	tegra_tsc_init();

	tegra_clk_init();

	debug_early_init();

	spe_pm_init();

	tegra_hsp_init(&tegra_hsp_id_aon);

	/*
	 * Late init task initialization for communicating with BPMP
	 * and then initializing IVC with CCPLEX
	 */
	late_init_sem = xSemaphoreCreateBinary();
	if (!late_init_sem) {
		error_hook("Couldn't create semaphore!");
		goto skip_late_init;
	}
	if (xTaskCreate(late_init_task, "late_init", 256, NULL, 3, NULL) != pdPASS) {
		error_hook("Couldn't create late_init_task");
	} else {
		irq_set_handler(MB1_DB_IRQ, late_init_db_irq, NULL);
		irq_enable(MB1_DB_IRQ);
	}

skip_late_init:
	if (bpmp_ipc_init())
		printf("bpmp_ipc_init() failed\r\n");

#if defined(ENABLE_GPCDMA_FUNC)
	tegra_gpcdma_init(&tegra_gpcdma_id_aon);
#endif

#if defined(ENABLE_GPIO_APP)
	/* gpio_chip_register should be done only once per gpio chip id */
	if (gpio_chip_register(TEGRA_AON_GPIO_CHIP_ID, &tegra_gpio_ops_aon,
				 &tegra_gpio_id_aon))
		error_hook("gpio_chip_register failed\r\n");
	gpio_app_init();
#endif

#if defined(ENABLE_TIMER_APP)
	timer_app_init();
#endif

#if defined(ENABLE_I2C_APP)
	i2c_app_init();
#endif

#if defined(ENABLE_UART_APP)
	uart_app_init();
#endif

#if defined(ENABLE_SPI_APP)
	spi_app_init();
#endif

#if defined(ENABLE_GTE_APP)
	gte_app_init();
#endif

#if defined(ENABLE_CAN_APP)
	can_app_init();
#endif

#if defined(ENABLE_AODMIC_APP)
	aodmic_app_init();
#endif

	/* Start the tasks and timer running. */
	vTaskStartScheduler();

	for( ;; )
		;

	return 0;
}

/* Referenced from FreeRTOSConfig.h */
void setup_timer_interrupt(void)
{
	tegra_tke_set_up_tick(&tegra_tke_id_timer0,
				TIMER_CLK_SRC,
				configCPU_CLOCK_HZ / configTICK_RATE_HZ);
}

/* Referenced from freertos */
void vApplicationStackOverflowHook(TimerHandle_t pxTask, signed char *pcTaskName)
{
	(void)pcTaskName;
	(void)pxTask;

	printf("Stack Overflow");
	taskDISABLE_INTERRUPTS();
	for( ;; )
		;
}
