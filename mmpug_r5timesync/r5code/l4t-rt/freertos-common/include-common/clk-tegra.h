/*
 * Copyright (c) 2014-2017, NVIDIA CORPORATION.  All rights reserved.
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

/*
 * This header defines a standard simple interface by which clocks can be
 * manipulated. The intent is to provide the same API on all SoCs and clocks,
 * even though the clock register layout may be quite different across those
 * variations. It is quite likely that multiple implementations of this API
 * will exist to cater for the SoC-to-SoC register differences.
 */

#ifndef _CLK_TEGRA_H_
#define _CLK_TEGRA_H_

#include <stdint.h>

/*
 * This is the handle or ID of a clock as used by the clock API.
 *
 * Each clock is identified by a pointer to a configuration structure so that
 * only configuration data for the exact set of clocks that is referenced is
 * included in the final binary.
 *
 * Each SoC supports a different set of clocks. The exact set of legal clock
 * IDs is defined by the SoC-specific <clk-tegra-hw.h>.
*/
struct tegra_clk;

/*
 * This is the handle or ID of a parent clock as used by the clock API.
 *
 * This handle is similar to that for struct tegra_clk. However, parent clocks
 * cannot be enabled or disabled. They can only have their clock rates set, so
 * that other clocks sourced by these clocks can have divisors set up correctly.
 *
 * Each SoC supports a different set of clocks. The exact set of legal clock
 * IDs is defined by the SoC-specific <clk-tegra-hw.h>.
*/
struct tegra_parent_clk;

/*
 * This is the handle or ID of a clock as used by the clock API.
 */
struct tegra_rst;

/*
 * Initialize the clock API.
 * For example, this may read/calculate the rates of clocks that the API
 * does not control, but uses as a parent for clocks that it does control.
 *
 * Returns 0 on success, other values on failure.
 */
int tegra_clk_init(void);

/*
 * Enable or disable a clock.
 *
 * Returns 0 on success, other values on failure.
 */
int tegra_clk_enable(const struct tegra_clk *clk);
int tegra_clk_disable(const struct tegra_clk *clk);

/*
 * Assert or release a reset signal.
 *
 * Returns 0 on success, other values on failure.
 */
int tegra_clk_reset_set(const struct tegra_rst *rst);
int tegra_clk_reset_clear(const struct tegra_rst *rst);

/*
 * Assert, delay, and release a reset signal.
 *
 * Returns 0 on success, other values on failure.
 */
int tegra_clk_reset_pulse(const struct tegra_rst *rst, uint32_t delay_us);

/*
 * Sets the rate of a clock.
 *
 * Returns 0 on success, other values on failure.
 */
int tegra_clk_set_rate(const struct tegra_clk *clk, uint32_t rate_hz);

/*
 * Sets the rate of a parent clock.
 *
 * Depending on the implementation, may not actually change the clock rate.
 *
 * Returns 0 on success, other values on failure.
 */
int tegra_parent_clk_set_rate(struct tegra_parent_clk *clk, uint32_t rate_hz);
#endif
