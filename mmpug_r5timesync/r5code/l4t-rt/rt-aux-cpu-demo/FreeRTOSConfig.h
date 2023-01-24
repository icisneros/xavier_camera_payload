/* Copyright (c) 2015-2016, NVIDIA CORPORATION. All rights reserved.
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

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include <artimer.h>

#define configUSE_PREEMPTION				1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION		0
#define configUSE_TICKLESS_IDLE				0
#define configCPU_CLOCK_HZ				1000000
#define configTICK_RATE_HZ				1000
#define configMAX_PRIORITIES				32
#define configMINIMAL_STACK_SIZE			128
#define configMAX_TASK_NAME_LEN				16
#define configUSE_16_BIT_TICKS				0
#define configIDLE_SHOULD_YIELD				1
#define configUSE_MUTEXES				1
#define configUSE_RECURSIVE_MUTEXES			1
#define configUSE_COUNTING_SEMAPHORES			1
#define configUSE_ALTERNATIVE_API			0 /* Deprecated! */
#define configQUEUE_REGISTRY_SIZE			10
#define configUSE_QUEUE_SETS				0
#define configUSE_TIME_SLICING				0
#define configUSE_NEWLIB_REENTRANT			1
#define configENABLE_BACKWARD_COMPATIBILITY		0

/* Hook function related definitions. */
#define configUSE_IDLE_HOOK				0
#define configUSE_TICK_HOOK				0
#define configCHECK_FOR_STACK_OVERFLOW			1
#define configUSE_MALLOC_FAILED_HOOK			0

/* Run time and task stats gathering related definitions. */
#define configGENERATE_RUN_TIME_STATS			0
#define configUSE_TRACE_FACILITY			0
#define configUSE_STATS_FORMATTING_FUNCTIONS		0

/* Co-routine definitions. */
#define configUSE_CO_ROUTINES				0
#define configMAX_CO_ROUTINE_PRIORITIES			0

/* Software timer related definitions. */
#define configUSE_TIMERS				1
#define configTIMER_TASK_PRIORITY			(configMAX_PRIORITIES - 1)
#define configTIMER_QUEUE_LENGTH			10
#define configTIMER_TASK_STACK_DEPTH			configMINIMAL_STACK_SIZE

#define configASSERT( x ) if( ( x ) == pdFALSE ) { taskDISABLE_INTERRUPTS(); for( ;; ); }

/* Set the following definitions to 1 to include the API function, or zero to exclude the API function. */
#define INCLUDE_vTaskPrioritySet			1
#define INCLUDE_uxTaskPriorityGet			1
#define INCLUDE_vTaskDelete				1
#define INCLUDE_vTaskSuspend				1
#define INCLUDE_xTaskResumeFromISR			1
#define INCLUDE_vTaskDelayUntil				1
#define INCLUDE_vTaskDelay				1
#define INCLUDE_xTaskGetSchedulerState			1
#define INCLUDE_xTaskGetCurrentTaskHandle		1
#define INCLUDE_uxTaskGetStackHighWaterMark		0
#define INCLUDE_xTaskGetIdleTaskHandle			0
#define INCLUDE_xTimerGetTimerDaemonTaskHandle		0
#define INCLUDE_pcTaskGetTaskName			0
#define INCLUDE_eTaskGetState				0
#define INCLUDE_xEventGroupSetBitFromISR		1
#define INCLUDE_xTimerPendFunctionCall			1

#ifndef __ASSEMBLER__
extern void setup_timer_interrupt(void);
#define configSETUP_TICK_INTERRUPT() setup_timer_interrupt()

#endif

#define configCLEAR_TICK_INTERRUPT
#define configACKNOWLEDGE_TICK_INTERRUPT

#endif /* FREERTOS_CONFIG_H */
