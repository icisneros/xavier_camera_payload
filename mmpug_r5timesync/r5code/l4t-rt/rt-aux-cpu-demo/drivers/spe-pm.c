/* Copyright (c) 2015-2018, NVIDIA CORPORATION.  All rights reserved.
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

#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>
#include <event_groups.h>

#include <address_map_new.h>
#include <araopm.h>
#include <artke_top.h>
#include <nvrm_drf.h>

#include <debug.h>

#include <irqs.h>
#include <irqs-hw.h>
#include <cache.h>
#include <delay.h>
#include <err-hook.h>
#include <macros.h>
#include <reg-access.h>
#include <spe-pm.h>
#include <bpmp-ipc.h>
#include <bpmp-ipc-protocol.h>
#include <spe-vic.h>
#include <wake-tegra.h>
#include <wake-tegra-hw.h>

#include <dbgprintf.h>

#if defined(ENABLE_COMBINED_UART_SERVER)
#include <combined-uart.h>
#endif

enum scx_state {
	SC0,
	SC7,
};

static enum scx_state current_soc_state = SC0;

static void wake_trigger_soc_wake(void)
{
	tegra_wake_trigger_wake_event(&tegra_wake_id_wake);
}

/*
 * FreeRTOS doesn't have condition variables. Here, we're using event groups
 * to have similar functionality. Instead of a mutex, we're disabling/enabling
 * interrupts for atomicity. Signal and wait functions for condition variables
 * are replaced by xEventGroupWaitBits and xEventGroupSetBits, respectively.
 *
 * This relies on the fact that this port can context switch when interrupts are
 * disabled. The interrupt disabled/enabled state is saved and restored over a
 * context switch. The end result acts similar to atomically dropping and
 * reacquiring a mutex when calling cond_wait.
 */
static EventGroupHandle_t dram_events;
static int num_dram_users = 0;
static bool dram_is_available = true;
static bool wake_triggered = false;

#define EVENT_DRAM_USABLE_BIT (1 << 0)
#define EVENT_DRAM_ACCESS_COMPLETE_BIT (1 << 1)

#define EVENT_DRAM_ALL_BITS (EVENT_DRAM_USABLE_BIT | \
			     EVENT_DRAM_ACCESS_COMPLETE_BIT)


bool try_request_dram_access_from_isr(void)
{
	/*
	 * This assumes that we don't have any nested interrupts, i.e all
	 * interrupts are masked during the execution of this ISR.
	 */
	bool ret = dram_is_available;

	if (ret) {
		num_dram_users++;
		xEventGroupClearBitsFromISR(dram_events,
				EVENT_DRAM_ACCESS_COMPLETE_BIT);
	}

	return ret;
}

bool try_request_dram_access(void)
{
	bool ret;

	portENTER_CRITICAL();

	ret = dram_is_available;
	if (ret) {
		num_dram_users++;
		xEventGroupClearBits(dram_events,
				EVENT_DRAM_ACCESS_COMPLETE_BIT);
	}

	portEXIT_CRITICAL();
	return ret;
}

void request_dram_access(void)
{
	portENTER_CRITICAL();
	num_dram_users++;
	xEventGroupClearBits(dram_events, EVENT_DRAM_ACCESS_COMPLETE_BIT);
	while (!dram_is_available) {
		if (wake_triggered) {
			xEventGroupWaitBits(dram_events, EVENT_DRAM_USABLE_BIT,
					pdFALSE, pdFALSE, portMAX_DELAY);
			continue;
		}

		switch (current_soc_state) {
		case SC7:
			wake_trigger_soc_wake();
			wake_triggered = true;
			break;

		case SC0:
		default:
			error_hook("Invalid SoC state!");
		}
	}
	portEXIT_CRITICAL();
}

bool dram_access_complete_from_isr(BaseType_t *woken)
{
	/*
	 * This assumes that we don't have any nested interrupts, i.e all
	 * interrupts are masked during the execution of this ISR.
	 */
	if (num_dram_users <= 0) {
		error_hook("Unbalanced or too many calls to dram_access_complete*");
		return false;
	}

	if (--num_dram_users != 0)
		return true;

	if (xEventGroupSetBitsFromISR(dram_events,
			EVENT_DRAM_ACCESS_COMPLETE_BIT, woken) == pdFAIL) {
		num_dram_users++;
		return false;
	}

	return true;
}

void dram_access_complete(void)
{
	portENTER_CRITICAL();

	if (num_dram_users <= 0) {
		error_hook("Unbalanced or too many calls to dram_access_complete*");
	} else if (--num_dram_users == 0) {
		xEventGroupSetBits(dram_events, EVENT_DRAM_ACCESS_COMPLETE_BIT);
	}
	portEXIT_CRITICAL();
}

/* Interrupts should be disabled before calling this function */
static void block_dram_access_from_isr(void)
{
	xEventGroupClearBits(dram_events, EVENT_DRAM_USABLE_BIT);

	while (num_dram_users > 0)
		xEventGroupWaitBits(dram_events, EVENT_DRAM_ACCESS_COMPLETE_BIT,
				pdTRUE, pdFALSE, portMAX_DELAY);

	dram_is_available = false;
}

/* Interrupts should be disabled before calling this function */
static void allow_dram_access_from_isr(void)
{
	dram_is_available = true;
	wake_triggered = false;
	xEventGroupSetBits(dram_events, EVENT_DRAM_USABLE_BIT);
}

void spe_prepare_enter_sc7(void)
{
	portENTER_CRITICAL();
	dbgprintf("spe entering sc7\r\n");
	block_dram_access_from_isr();
	current_soc_state = SC7;
	portEXIT_CRITICAL();

	pm_suspend_sc7();

#if defined(ENABLE_COMBINED_UART_SERVER)
	combined_uart_suspend();
#endif

	if (get_cache_power_status() == PM_RAM_ON)
		cache_disable();
	pm_turn_off_caches();
	bpmp_ipc_send(BPMP_IPC_MSG_SC7_ENTRY_ACK);
}

void spe_exit_sc7(void)
{
#if defined(ENABLE_COMBINED_UART_SERVER)
	combined_uart_resume();
#endif

	dbgprintf("spe exiting sc7\r\n");
	pm_turn_on_caches();
	if (get_cache_power_status() == PM_RAM_ON)
		cache_enable();

	pm_resume_sc7();

	portENTER_CRITICAL();
	allow_dram_access_from_isr();
	current_soc_state = SC0;
	portEXIT_CRITICAL();
}

int spe_pm_init(void)
{
	dram_events = xEventGroupCreate();
	if (!dram_events) {
		error_hook("Couldn't create event queue for dram access.");
		return 1;
	}

	xEventGroupClearBits(dram_events, EVENT_DRAM_ALL_BITS);

	pm_hw_init();

	return 0;
}
