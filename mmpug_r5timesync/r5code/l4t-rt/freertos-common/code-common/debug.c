/*
 * Copyright (c) 2014-2017 NVIDIA CORPORATION.  All rights reserved.
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

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>

#include <aruart.h>

#include <clk-tegra.h>
#include <clk-tegra-hw.h>
#include <debug.h>
#include <debug-hw.h>
#include <reg-access.h>
#include <macros.h>
#include <printf-isr.h>

#define TEGRA_DBG_UART_TX	(TEGRA_DBG_UART_BASE + UART_THR_DLAB_0_0)
#define TEGRA_DBG_UART_LSR	(TEGRA_DBG_UART_BASE + UART_LSR_0)
#define TEGRA_DBG_UART_LSR_THRE	(UART_LSR_0_THRE_EMPTY << UART_LSR_0_THRE_SHIFT)
#define TEGRA_DBG_UART_LSR_TMTY	(UART_LSR_0_TMTY_EMPTY << UART_LSR_0_TMTY_SHIFT)

#define TEGRA_DBG_UART_LCR	(TEGRA_DBG_UART_BASE + UART_LCR_0)
#define TEGRA_DBG_UART_LCR_WD_SIZE \
	(UART_LCR_0_WD_SIZE_WORD_LENGTH_8 << UART_LCR_0_WD_SIZE_SHIFT)
#define TEGRA_DBG_UART_LCR_PAR \
	(UART_LCR_0_PAR_NO_PARITY << UART_LCR_0_PAR_SHIFT)
#define TEGRA_DBG_UART_LCR_STOP \
	(UART_LCR_0_STOP_DISABLE << UART_LCR_0_STOP_SHIFT)
#define TEGRA_DBG_UART_LCR_DLAB \
	(UART_LCR_0_DLAB_ENABLE << UART_LCR_0_DLAB_SHIFT)

#define TEGRA_DBG_UART_IER_DLAB_IE_RHR \
	(UART_IER_DLAB_0_0_IE_RHR_ENABLE << UART_IER_DLAB_0_0_IE_RHR_SHIFT)

#define TEGRA_DBG_UART_IIR_FCR	(TEGRA_DBG_UART_BASE + UART_IIR_FCR_0)
#define TEGRA_DBG_UART_IIR_FCR_EN_FIFO \
	(UART_IIR_FCR_0_FCR_EN_FIFO_ENABLE << UART_IIR_FCR_0_FCR_EN_FIFO_SHIFT)
#define TEGRA_DBG_UART_IIR_FCR_RX_CLR \
	(UART_IIR_FCR_0_RX_CLR_CLEAR << UART_IIR_FCR_0_RX_CLR_SHIFT)
#define TEGRA_DBG_UART_IIR_FCR_TX_CLR \
	(UART_IIR_FCR_0_TX_CLR_CLEAR << UART_IIR_FCR_0_TX_CLR_SHIFT)

#define TEGRA_DBG_UART_THR_DLAB	(TEGRA_DBG_UART_BASE + UART_THR_DLAB_0_0)
#define TEGRA_DBG_UART_IER_DLAB	(TEGRA_DBG_UART_BASE + UART_IER_DLAB_0_0)

static bool inited = false;

void dbg_init(void)
{
	uint32_t lcr = TEGRA_DBG_UART_LCR_WD_SIZE | TEGRA_DBG_UART_LCR_PAR |
		TEGRA_DBG_UART_LCR_STOP;

	/* Setup UART to 8n1 */
#ifdef TEGRA_DBG_UART_CLK
	tegra_clk_set_rate(TEGRA_DBG_UART_CLK, TEGRA_DBG_UART_BAUD * 16);
#if !defined(_NV_BUILD_FPGA_) && !defined(_NV_BUILD_LINSIM_)
	tegra_clk_enable(TEGRA_DBG_UART_CLK);
	tegra_clk_reset_clear(TEGRA_DBG_UART_RST);
#endif

	writel(lcr | TEGRA_DBG_UART_LCR_DLAB, TEGRA_DBG_UART_LCR);
	writel(0x1, TEGRA_DBG_UART_THR_DLAB); /* Divider LSB */
	writel(0x0, TEGRA_DBG_UART_IER_DLAB); /* Divider MSB */
#endif
	writel(lcr, TEGRA_DBG_UART_LCR);
	writel(0x0, TEGRA_DBG_UART_IIR_FCR);
	writel(TEGRA_DBG_UART_IER_DLAB_IE_RHR, TEGRA_DBG_UART_IER_DLAB);
	writel(TEGRA_DBG_UART_IIR_FCR_EN_FIFO,TEGRA_DBG_UART_IIR_FCR);
	writel(TEGRA_DBG_UART_IIR_FCR_EN_FIFO |
	       TEGRA_DBG_UART_IIR_FCR_RX_CLR |
	       TEGRA_DBG_UART_IIR_FCR_TX_CLR, TEGRA_DBG_UART_IIR_FCR);
	/* WAR to clear any pending interrupts after clearing the FIFOs */
	writel(0, TEGRA_DBG_UART_IER_DLAB);
	writel(TEGRA_DBG_UART_IER_DLAB_IE_RHR, TEGRA_DBG_UART_IER_DLAB);

	inited = true;
}

void dbg_3rdparty_init(void)
{
	inited = true;
}

void dbg_putc(char c)
{
	if (!inited)
		return;

	while (!(readl(TEGRA_DBG_UART_LSR) & TEGRA_DBG_UART_LSR_THRE))
		;
	writel(c, TEGRA_DBG_UART_TX);
}

void dbg_putd(const char *d, uint32_t len)
{
	uint32_t i;
	for (i = 0; i < len; i++)
		dbg_putc(*d++);
}

void dbg_puts(const char *s)
{
	while (*s)
		dbg_putc(*s++);
}

static char nibble_to_hex(uint32_t n)
{
	if (n < 10)
		return '0' + n;
	return 'a' + n - 10;
}

void dbg_puthex8(uint32_t n)
{
	dbg_puts("0x");
	dbg_putc(nibble_to_hex((n >> 28) & 0xf));
	dbg_putc(nibble_to_hex((n >> 24) & 0xf));
	dbg_putc(nibble_to_hex((n >> 20) & 0xf));
	dbg_putc(nibble_to_hex((n >> 16) & 0xf));
	dbg_putc(nibble_to_hex((n >> 12) & 0xf));
	dbg_putc(nibble_to_hex((n >>  8) & 0xf));
	dbg_putc(nibble_to_hex((n >>  4) & 0xf));
	dbg_putc(nibble_to_hex( n        & 0xf));
}

/* This lookup table avoids the integer division (div /= 10) */
static uint32_t divisors[] = {1, 10, 100, 1000, 10000, 100000, 1000000,
				10000000, 100000000, 1000000000};

void dbg_putdec(uint32_t n)
{
	int i;
	bool keep_print = false;
	uint32_t div;
	uint32_t res;

	if (n == 0) {
		dbg_putc('0');
		return;
	}

	for (i = ARRAY_SIZE(divisors) - 1; i >= 0; i--) {
		div = divisors[i];
		if (n >= div) {
			keep_print = true;
			res = n / div;
			dbg_putc(res + '0');
			n -= res * div;
		} else if (keep_print) {
			dbg_putc('0');
		}
	}
}

int vprintf_isr(const char *fmt, va_list ap)
{
	char msg[PRINTF_ISR_BUFSIZE];
	int ret;

	ret = vsnprintf(msg, PRINTF_ISR_BUFSIZE, fmt, ap);

	if (ret > 0)
	{
		ret = (ret > PRINTF_ISR_BUFSIZE) ? PRINTF_ISR_BUFSIZE : ret;
		write(2, msg, ret);
	}

	return ret;
}

int printf_isr(const char *fmt, ...)
{
	va_list args;
	int ret;

	va_start(args, fmt);
	ret = vprintf_isr(fmt, args);
	va_end(args);

	return ret;
}

void dbg_flush(void)
{
	while (!(readl(TEGRA_DBG_UART_LSR) & TEGRA_DBG_UART_LSR_TMTY))
		;
}
