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

/* Note: Follow doc/gte-app.txt for more information */

#include <delay.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <macros.h>
#include <err-hook.h>
#include <gte-tegra.h>
#include <gte-tegra-hw.h>
#include <printf-isr.h>
#include <gpio-client.h>
#include <gpio-provider.h>
#include "gte-app.h"
#include "gpio-aon.h"

#define GTE_TEST_FIFO_OCCUPANCY         1

static void gte_irq_callback(void *data, struct tegra_gte_ts *gte_ts)
{
        const char *evname;

        if (gte_ts->slice == 1 &&
            gte_ts->bit_index == NV_AON_GTE_SLICE1_IRQ_GPIO)
                evname = "GPIO IRQ";
        else if (gte_ts->slice == 2 &&
            gte_ts->bit_index == NV_AON_GTE_SLICE2_IRQ_GPIO_2)
                evname = "GPIO_APP_IN";
        else
                evname = "Other";

        printf_isr("Slice Id: %d, Event Id: %d (%s), Edge = %s, "
                   "Time stamp = %x%08x\r\n", gte_ts->slice, gte_ts->bit_index,
                   evname, gte_ts->bit_dir ? "rising" : "falling",
                   (unsigned int)(gte_ts->tsc >> 32),
                   (unsigned int)(gte_ts->tsc));
}

void gte_app_init(void)
{
        /* Enable GPIO IRQ GTE timestamp */
        tegra_gte_slice_set_enable_mask(&tegra_gte_id_aon,
                                        1, /* slice number 1 */
                                        BIT(NV_AON_GTE_SLICE1_IRQ_GPIO));
        /* Enable GPIO_APP_IN GPIO GTE timestamp */
        tegra_gte_slice_set_enable_mask(&tegra_gte_id_aon,
                                        2, /* slice number 2 */
                                        BIT(NV_AON_GTE_SLICE2_IRQ_GPIO_2));

        int ret = tegra_gte_setup(&tegra_gte_id_aon, GTE_TEST_FIFO_OCCUPANCY,
                                  gte_irq_callback, 0, 0, 0);
        if (ret)
                error_hook("GTE setup failed\r\n");

        /* Enable timestamping on GPIO_APP_IN at GPIO controller */
        ret = gpio_enable_timestamp(GPIO_APP_IN);
        if (ret)
                error_hook("gpio_enable_timestamp -failed\r\n");
}
