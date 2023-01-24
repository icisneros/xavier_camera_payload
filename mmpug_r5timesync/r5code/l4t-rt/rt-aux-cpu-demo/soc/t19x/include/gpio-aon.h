/* Copyright (c) 2019-2020, NVIDIA CORPORATION. All rights reserved.
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

#ifndef _GPIO_AON_T19X_H
#define _GPIO_AON_T19X_H

#include <tegra194-gpio-hw.h>

#define TEGRA_AON_GPIO_CHIP_ID 0
#define TEGRA_AON_GPIO_ID(bank, gpio) \
		gpio_global_id(TEGRA_AON_GPIO_CHIP_ID, TEGRA_GPIO(bank, gpio))

#if defined(ENABLE_SPE_FOR_NX)
#define GPIO_APP_OUT TEGRA_AON_GPIO_ID(CC, 4) /* GPIO_CC 4, pin 15, J12 */
#define GPIO_APP_IN  TEGRA_AON_GPIO_ID(DD, 0) /* GPIO_DD 0, pin 27, J12 */
#else
#define GPIO_APP_OUT TEGRA_AON_GPIO_ID(BB, 0) /* GPIO_BB 0, pin 16, J30 */
#define GPIO_APP_IN  TEGRA_AON_GPIO_ID(BB, 1) /* GPIO_BB 1, pin 32, J30 */
#endif

#endif
