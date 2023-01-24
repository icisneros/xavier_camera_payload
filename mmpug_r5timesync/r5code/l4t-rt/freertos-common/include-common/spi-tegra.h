/*
 * Copyright (c) 2015-2016, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _SPI_TEGRA_H_
#define _SPI_TEGRA_H_

#include <stdbool.h>
#include <stdint.h>

struct tegra_spi_id;

/*
 * Please refer to Tegra SPI HW manual for the meaning of following SPI timing
 * settings
 * chip_select:			specify which CS pin for this structure
 * cs_setup_clk_count:		CS pin setup clock count
 * cs_hold_clk_count:		CS pin hold clock count
 * cs_inactive_cycles:		CS pin inactive clock count
 * set_rx_tap_delay:		specify if the SPI device need to set RX tap delay
 * spi_max_clk_rate:		specify the default clock rate of SPI client
 * spi_no_dma:			flag to indicate pio or dma mode
 */
struct tegra_spi_client_setup {
	uint32_t spi_max_clk_rate;
	int cs_setup_clk_count;
	int cs_hold_clk_count;
	int cs_inactive_cycles;
	uint8_t chip_select;
	bool set_rx_tap_delay;
	bool spi_no_dma;
};

/*
 * Following three settings are GPCDMA related
 * dma_id:		DMA ID of the associated GPCDMA channel
 * dma_channel:		specify which GPCDMA channels to use for tx, rx
 * dma_slave_req:	specify which GPCDMA slave req type is for this DMA
 *			request. Please check argpcdma*.h for appropriate
 *			setting
 * spi_max_clk_rate:	specify the max clock rate of SPI controller
 */
struct tegra_spi_master_init {
	struct tegra_gpcdma_id *dma_id;
	struct {
		int tx, rx;
	} dma_channel;
	uint32_t spi_max_clk_rate;
	uint8_t dma_slave_req;
};

/*
 * flags:		Indicate first/last message.
 * tx_buf:		data to write to SPI device, or NULL if this is RX transfer
 *			the buffer memory need to be aligned for DMA transfer
 * rx_buf:		buffer to hold read data, or NULL if this is TX transfer
 *			the buffer memory need to be aligned for DMA transfer
 * len:			size of rx or tx buffer in bytes
 * tx_nbits:		number of bits used for writing
 * rx_nbits:		number of bits used for reading
 * bits_per_word:	select bits_per_word
 *
 * spi_clk_rate:	specify clock rate for current transfer
 *
 * When SPI can transfer in 1x,2x or 4x. It can get this tranfer information
 * from device through tx_nbits and rx_nbits. In Bi-direction, these
 * two should both be set. User can set transfer mode with SPI_NBITS_SINGLE(1x)
 * SPI_NBITS_DUAL(2x) and SPI_NBITS_QUAD(4x) to support these three transfer.
 */
struct tegra_spi_xfer {
	uint16_t flags;
#define TEGRA_SPI_XFER_FIRST_MSG	3
#define TEGRA_SPI_XFER_LAST_MSG		4
#define TEGRA_SPI_XFER_HANDLE_CACHE	5
	const void *tx_buf;
	void *rx_buf;
	unsigned len;
	uint8_t chip_select;

	unsigned tx_nbits:3;
	unsigned rx_nbits:3;
#define TEGRA_SPI_NBITS_SINGLE	0x01			/* 1bit transfer */
#define TEGRA_SPI_NBITS_DUAL	0x02			/* 2bits transfer */
#define TEGRA_SPI_NBITS_QUAD	0x04			/* 4bits transfer, supported since T186 */

	uint8_t bits_per_word;

	uint16_t mode;
#define TEGRA_SPI_CPHA		0x01			/* clock phase */
#define TEGRA_SPI_CPOL		0x02			/* clock polarity */
#define TEGRA_SPI_MODE_0	(0 | 0)			/* (original MicroWire) */
#define TEGRA_SPI_MODE_1	(0 | TEGRA_SPI_CPHA)
#define TEGRA_SPI_MODE_2	(TEGRA_SPI_CPOL | 0)
#define TEGRA_SPI_MODE_3	(TEGRA_SPI_CPOL | TEGRA_SPI_CPHA)
#define TEGRA_SPI_CS_HIGH	0x04			/* chipselect active high */
#define TEGRA_SPI_LSB_FIRST	0x08			/* per-word bits-on-wire */
#define TEGRA_SPI_3WIRE		0x10			/* SI/SO signals shared */
#define TEGRA_SPI_LSBYTE_FIRST	0x1000			/* per-word bytes-on-wire */
	uint32_t spi_clk_rate;
};

/*
 * Initialize and set the parameters of the requested SPI device
 *
 * Parameters:
 * id:		SPI controller instance
 * setting:	SPI device setting
 *
 * Returns:
 * 0: Success
 * Negative: error
 */
int tegra_spi_setup(struct tegra_spi_id *id, struct tegra_spi_client_setup *setting);

/*
 * Initialize and set up the controller configuration
 *
 * Parameters:
 * id:		SPI controller instance
 * setting:	SPI controller setting
 *
 * Returns:
 * 0: Success
 * Negative: error
 */
int tegra_spi_init(struct tegra_spi_id *id, struct tegra_spi_master_init *setting);

/*
 * Execute SPI transfer(s) on the specified SPI device
 *
 * This function can transfer one or multiple SPI transfers
 * During the function, CS pin will be kept asserted
 * After the function is done, CS pin will be de-asserted
 *
 * The code that submits tegra_spi_xfer to the lower layers is responsible for
 * managing its memory.
 *
 * Parameters:
 * id:		SPI controller instance
 * xfer:	SPI transfer(s)
 *
 * Returns:
 * 0: Success
 * Negative: error
 */
int tegra_spi_transfer(struct tegra_spi_id *id, struct tegra_spi_xfer *xfer);

/*
 * SPI IRQ handler. Call this whenever SPI controller raises an IRQ.
 * Each SPI bus should has a separate IRQ number.
 *
 * Parameter:
 * data:	SPI controller instance
 */
void tegra_spi_irq(void *data);

/*
 * This function is used to check if the controller is busy and if not
 * disables the clock.
 *
 * Parameter:
 * id:		SPI controller instance
 */
int tegra_spi_suspend(struct tegra_spi_id *id);

/*
 * This function is used to enable the clock for the SPI controller.
 *
 * Parameter:
 * id:		SPI controller instance
 */
int tegra_spi_resume(struct tegra_spi_id *id);

#endif
