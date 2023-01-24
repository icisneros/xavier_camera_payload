/* Copyright (c) 2015-2018 NVIDIA CORPORATION.  All rights reserved.
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

#ifndef __SPE_PM_H
#define __SPE_PM_H

#include <FreeRTOS.h>
#include <portmacro.h>

#include <stdbool.h>
#include <config.h>

enum pm_ram_pg_status {
	PM_RAM_OFF,
	PM_RAM_ON,
	PM_RAM_ERROR
};

int spe_pm_init(void);

void pm_hw_init(void);
void pm_suspend_sc7(void);
void pm_resume_sc7(void);
void pm_turn_off_caches(void);
void pm_turn_on_caches(void);

/*
 * Called after BPMP notifies SPE that the SoC is going to SC7 state
 */
void spe_prepare_enter_sc7(void);

/*
 * Called after BPMP notifies SPE that the SoC is going to SC8 state
 */
void spe_prepare_enter_sc8(void);

/*
 * Called after BPMP notifies SPE that the SoC is in SC0 after SC7/8 states
 */
void spe_exit_sc7(void);

enum pm_ram_pg_status get_cache_power_status(void);

/*
 * Tries to access dram, doesn't block. Does not try to wake dram up on failure.
 * Returns true on success. This should not be called from an ISR, see
 * try_request_dram_access_from_isr() for that.
 */
bool try_request_dram_access(void);

/*
 * Tries to access dram, doesn't block. Does not try to wake dram up on failure.
 * Returns true on success. This is the ISR-safe version.
 */
bool try_request_dram_access_from_isr(void);

/*
 * Requests access to dram. If available, it will return immediately. Else, it
 * will block until dram becomes available. It will also send a signal to BPMP
 * to wake the SoC up.
 */
void request_dram_access(void);

/*
 * This function should be called to indicate that a task is done using dram or
 * other SoC resources. This should be called once for each call to
 * request_dram_access, or a succesful call to try_request_dram_access.
 * This should not be called from an ISR. See dram_access_complete_from_isr()
 * for that.
 */
void dram_access_complete(void);

/*
 * This function should be called to indicate that a task is done using dram or
 * other SoC resources. This should be called once for each call to
 * request_dram_access, or a succesful call to try_request_dram_access.
 * This is the ISR-safe version, however, it can fail if the timer daemon queue
 * is full.
 * Returns true on success.
 */
bool dram_access_complete_from_isr(BaseType_t *higher_prio_task_woken);

#endif
