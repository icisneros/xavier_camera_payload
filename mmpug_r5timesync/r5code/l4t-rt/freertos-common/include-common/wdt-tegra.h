/*
 * Copyright (c) 2015-2017 NVIDIA CORPORATION.  All rights reserved.
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

#ifndef INCLUDED_WDT_TEGRA_H
#define INCLUDED_WDT_TEGRA_H

#include <stdbool.h>
#include <stdint.h>

struct tegra_wdt_id;
typedef void (*tegra_wdt_callback)(uint32_t status, void *data);

struct tegra_wdt_conf {
	uint8_t tmrsrc;
	uint8_t period;
	bool irq_en;
	bool fiq_en;
	bool remoteIrq_en;
	bool sys_dbg_rst_en;
	bool sys_por_rst_en;
	bool tsc_ref_en;
	uint8_t err_threshold;
	bool challenge_response;
	bool irq_ack_by_callback;
	bool fiq_ack_by_callback;
	tegra_wdt_callback irq_callback;
	void *irq_data;
	tegra_wdt_callback fiq_callback;
	void *fiq_data;
};

/*
 * Tegra WDT driver
 *
 * To use WDT driver,
 * 1. Call tegra_wdt_setup to setup watchdog timer first
 * 2. Call tegra_wdt_start to enable watchdog timer
 *
 * The wdt driver IRQ/FIQ handler will ack the watchdog automatically. If
 * you want to do it from the callback or from a scheduld task, set
 * irq_ack_by_callback/fiq_ack_by_callback.
 *
 * Note, tegra_wdt_setup can only be called if WDT is disabled
 * if WDT is enabled then call tegra_wdt_stop or tegra_wdt_stop_from_isr to
 * stop WDT counter first and then call tegra_wdt_setup to initialize WDT.
 */

/*
 * Initialize watchdog timer
 * Returns:
 *  0:           Success
 *  Non-zero:    Error
 */
int tegra_wdt_setup(const struct tegra_wdt_id *id, const struct tegra_wdt_conf *conf);

/*
 *  Start watchdog timer counter
 */
void tegra_wdt_start(const struct tegra_wdt_id *id);

/*
 * Acknowledge watchdog timer and reset its level
 */
void tegra_wdt_ack(const struct tegra_wdt_id *id);

/*
 * Disabled watchdog timer, this can be used from an ISR
 */
void tegra_wdt_stop_from_isr(const struct tegra_wdt_id *id);

/*
 * Disabled watchdog timer.
 */
void tegra_wdt_stop(const struct tegra_wdt_id *id);

/*
 * Watchdog timer interrupt handler
 */
void tegra_wdt_irq(void *id);

/*
 * Watchdog timer fast interrupt handler
 */
void tegra_wdt_fiq(void *id);

/*
 * Read watchdog timer status
 * Returns:
 *  Status:
 *   bit [0], Enabled: 1b when the counter is active,
 *                     0b when the counter is disabled
 *   bit [1], LocalIRQStatus: Current status of interrupt
 *   bit [2], LocalFIQStatus: Current status of FIQ
 *   bit [3], RemoteInterruptStatus: Current status of remote interrupt
 *   bit [11:4], CurrentCount: Current value of the counter
 *   bit [14:12], CurrentExpirationCount: Current count of expiration
 *                since last start operation
 *   bit [16], CurrentError: Current error reported to HSM
 */
uint32_t tegra_wdt_read_status(const struct tegra_wdt_id *id);

#endif /* INCLUDED_WDT_TEGRA_H */
