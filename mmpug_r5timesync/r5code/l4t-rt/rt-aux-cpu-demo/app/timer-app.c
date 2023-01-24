/*
 * Copyright (c) 2019, NVIDIA CORPORATION.  All rights reserved.
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

#include <timeserver.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <printf-isr.h>
#include <tke-tegra.h>
#include <tke-tegra-hw.h>
#include <gpio-client.h>
#include "timer-app.h"
#include <reg-access.h>
#include <bitops.h>
#include <address_map_new.h>





#include <gpio-provider.h>

/* gpio-aon.h has GPIO_APP* defines */
#include "gpio-aon.h"


/* Demo timer app which sets up 5 second periodic timer, modify below define to
 * adjust periodic value of the timer.
 */
#define TIMER2_PTV 					((uint32_t)(((double) timeserverdata.ticks_persecond) / 10.0))
#define IMU_ROLL_THRESH				(((double) timeserverdata.ticks_persecond) * 0.040)
#define TKE_TIMER_TMRCR_0_PTV		0x1fffffffU
#define TKE_TIMER_TMRCR_0			0x0
#define TKE_TIMER_TMRCR_0_PER		1<<30
#define TKE_TIMER_TMRCR_0_EN		1<<31
// #define PPS_ON_TIME					((uint32_t) (((double) timeserverdata.ticks_persecond / 10.0)+0.5))  	//100ms
// #define PPS_OFF_TIME				((uint32_t) (9.0*((double) timeserverdata.ticks_persecond / 10.0)+0.5))	//900ms
#define TKE_TIMER_TMRATR_0		0xC
#define TKE_TIMER_TMRATR_0_ATR		0x3fffffffU



static void timer2_callback(void *data)
{
	static uint64_t rising;
	static int32_t old_phase;
	static uint32_t count;

	

		if(count == 0){
			// Count 0 = top of second
			// Change to 1 if synced
			if(timeserverdata.synced>0)
			{
				gpio_set_value(GPIO_APP_OUT, 1);
				gpio_set_value(GPIO_APP_IN, 1);
			} else {
				printf_isr("No sync signal received from main computer. not outputting pulses\r\n");
			}

			// Get approximate time we asserted pin high
			rising = tegra_tke_get_tsc64();
			
			count++;
		} else if(count <9){
			// Not at the top of the second
			// Change to 0
			gpio_set_value(GPIO_APP_OUT, 0);
			gpio_set_value(GPIO_APP_IN, 0);
			count++;
		} else if(count >= 9){
			// Change to 0
			gpio_set_value(GPIO_APP_OUT, 0);
			gpio_set_value(GPIO_APP_IN, 0);

			// Find the nearest second we're in for the TSC, rising set just after rising edge
			rising = rising - (rising % timeserverdata.ticks_persecond);
			// We've removed our alignment to the start of TSC second
			// Add a full second for the start of next TSC second
			rising += timeserverdata.ticks_persecond;
			// Add old offset and we're at the start of the old RTC second
			rising += old_phase;
			// rising += (timeserverdata.ticks_persecond - timeserverdata.phaseoffset_ticks);

			uint32_t offset_inv = timeserverdata.ticks_persecond - timeserverdata.phaseoffset_ticks;
			int32_t phase_delta = (int32_t)offset_inv - old_phase;
			uint32_t phase_mag = abs(phase_delta);
			if(phase_delta > (int32_t)IMU_ROLL_THRESH){
				phase_delta = IMU_ROLL_THRESH;
				rising+=IMU_ROLL_THRESH;
			} else if(phase_mag > (int32_t)IMU_ROLL_THRESH) {
				phase_delta = 0L - (int32_t)IMU_ROLL_THRESH;
				rising-=IMU_ROLL_THRESH;
			} else if(phase_delta > 0L){
				rising+=phase_delta;
			} else if(phase_delta < 0L){
				rising-=phase_mag;
			} else{
				// Do nothing, safe to use these values
			}

			// Get current time
			uint64_t current = tegra_tke_get_tsc64();
			uint32_t rising32 = ((uint32_t)rising & TKE_TIMER_TMRATR_0_ATR);
			// Stupid ATR uses 30 bits [29:0] and treats this as signed :(
			uint32_t current_lo = (uint32_t) current;
			int32_t rising_signed = (int32_t)(rising32 << 2);
			int32_t current_lo_signed = (int32_t)(current_lo <<2);
			if(rising_signed <= current_lo_signed){
				// Wrap around, do nothing
			} else {
				// Set PTV to updated ticks_persecond
				tegra_tke_timer_writel(&tegra_tke_id_timer2, ((TIMER2_PTV - 1) & TKE_TIMER_TMRCR_0_PTV) | TKE_TIMER_TMRCR_0_EN | TKE_TIMER_TMRCR_0_PER, TKE_TIMER_TMRCR_0);
				// Write ATR to keep phase alignment after updating PTV
				tegra_tke_timer_writel(&tegra_tke_id_timer2, rising32, TKE_TIMER_TMRATR_0);
				// Save old phase only when written
				old_phase += phase_delta;
			}	
			
			
			count = 0;
	}
	

}

void timer_app_init(void)
{

	int val = gpio_direction_out(GPIO_APP_OUT, 0);
	if (val) {
		return;
	}
	val = gpio_direction_out(GPIO_APP_IN, 0);
	if (val) {
		return;
	}
	//Set PTV to max value when using ATR, from Nvidia forums
	//todo - why must periodic be set to true for atr target to function
	tegra_tke_set_up_timer(&tegra_tke_id_timer2, TEGRA_TKE_CLK_SRC_TSC_BIT0,
			       true, TIMER2_PTV, timer2_callback, 0);
	uint64_t temp_tsc = tegra_tke_get_tsc64();
	temp_tsc = temp_tsc - (temp_tsc % timeserverdata.ticks_persecond);
	// uint32_t tsc_lo = (uint32_t) temp_tsc;
	// uint32_t tsc_align = temp_tsc % timeserverdata.ticks_persecond;
	// (tsc_lo - tsc_align) = start of tsc second, add phaseoffset_ticks, plus one second since that time sync for this "second frame" may have past
	// uint32_t offset_ticks = timeserverdata.phaseoffset_ticks + get_AON_offset();		
	//get_AON_offset() here causes this not to flash lol
	uint32_t offset_inv = timeserverdata.ticks_persecond - timeserverdata.phaseoffset_ticks;
	uint32_t tsc_tmr_atr = (uint32_t)temp_tsc + offset_inv + timeserverdata.ticks_persecond;
	tsc_tmr_atr = tsc_tmr_atr & TKE_TIMER_TMRATR_0_ATR;
	// if(tsc_tmr_atr < TIMER2_PTV){
	// 	// we wrapped around, this will cause an immmediate interrupt
	// 	// Let first loop of callback hamdle sync
	// } else {
	// 	// Not going to wrap around, should be fine
		tegra_tke_timer_writel(&tegra_tke_id_timer2, (tsc_tmr_atr & TKE_TIMER_TMRATR_0_ATR), TKE_TIMER_TMRATR_0);
	// }	
	
}
