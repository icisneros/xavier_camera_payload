/*
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _GPIO_TEGRA_HW_PARAMS_H_
#define _GPIO_TEGRA_HW_PARAMS_H_

#include <irqs-hw.h>
#include <irqs-lic.h>

#define TEGRA_MAIN_GPIO_ID0_IRQ		(AON_LIC_IRQ_GPIO0)
#define TEGRA_MAIN_GPIO_ID1_IRQ		(AON_LIC_IRQ_GPIO1)
#define TEGRA_MAIN_GPIO_ID2_IRQ		(AON_LIC_IRQ_GPIO2)
#define TEGRA_MAIN_GPIO_ID3_IRQ		(AON_LIC_IRQ_GPIO3)
#define TEGRA_MAIN_GPIO_ID4_IRQ		(AON_LIC_IRQ_GPIO4)
#define TEGRA_MAIN_GPIO_ID5_IRQ		(AON_LIC_IRQ_GPIO5)
#define TEGRA_MAIN_GPIO_IRQ		(AON_LIC_IRQ_GPIO0)
#define TEGRA_AON_GPIO_IRQ		(NV_AON_INTERRUPT_GPIO + \
						NV_AON_INTERRUPT_VIC1_BASE)
#define TEGRA_GPIO_IRQ_STATUS_REG	GPIO_N_INTERRUPT_STATUS_G4_0

#endif
