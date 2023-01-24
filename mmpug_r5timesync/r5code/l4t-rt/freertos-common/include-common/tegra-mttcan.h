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

#ifndef __M_TTCAN_DEF
#define __M_TTCAN_DEF

#include <stdio.h>
#include <strings.h>

#include <errno.h>
#include <reg-access.h>
#include <err-hook.h>

#include <tegra-can.h>
#include <tegra-mttcan-regdefs.h>

#define CAN_STD_ID 11
#define CAN_EXT_ID 29

#define CAN_STD_ID_MASK 0x000007FFU
#define CAN_EXT_ID_MASK 0x1FFFFFFFU
#define CAN_ERR_MASK 0x1FFFFFFFU

#define CAN_FMT 0x80000000U	/*1= EXT/ 0 = STD */
#define CAN_RTR 0x40000000U	/* RTR */
#define CAN_ERR 0x20000000U	/* ERR/Data  message frame */

#define CAN_BRS_MASK 0xFE
#define CAN_ESI_MASK 0xFD
#define CAN_FD_MASK  0xFB
#define CAN_DIR_MASK 0xF7

#define CAN_BRS_FLAG 0x01
#define CAN_ESI_FLAG 0x02
#define CAN_FD_FLAG  0x04
#define CAN_DIR_RX   0x08

#define MTTCAN_RAM_SIZE 4096
#define MTTCAN_RAM_SIZE_WORDS 1024
#define CAN_WORD_IN_BYTES 4

/* ISO 11898-1 */
#define CAN_MAX_DLC 8
#define CAN_MAX_DLEN 8
#define CAN_ERR_DLC_DLEN 8

/* ISO 11898-7 */
#define CANFD_MAX_DLC 15
#define CANFD_MAX_DLEN 64

#define MAX_RX_ENTRIES 64

#define MAX_LEC 8

#define NUM_CAN_CONTROLLERS 2

/* Global Filter Confugration */
#define MTTCAN_GFC_ANFS_SHIFT	0x04
#define GFC_ANFS_RXFIFO_0	0x00
#define GFC_ANFS_RXFIFO_1	(0x01 << MTTCAN_GFC_ANFS_SHIFT)
#define GFC_ANFS_REJECT		(0x03 << MTTCAN_GFC_ANFS_SHIFT)

#define MTTCAN_GFC_ANFE_SHIFT	0x02
#define GFC_ANFE_RXFIFO_0	0x00
#define GFC_ANFE_RXFIFO_1	(0x01 << MTTCAN_GFC_ANFE_SHIFT)
#define GFC_ANFE_REJECT		(0x03 << MTTCAN_GFC_ANFE_SHIFT)

#define GFC_RRFS_REJECT		0x2
#define GFC_RRFE_REJECT		0x1

#define IE_RXF0_MASK  (0xF)
#define IE_RXF1_MASK  (0xF << 0x4)
#define MTTCAN_TX_EV_FIFO_INTR  (0xF << 12)

#define CAN_GLUE_ADDR 0x1000
#define CAN_SET_OFFSET	0x0
#define CAN_CLR_OFFSET	0x4

#define CAN_ILE_0	0x1
#define CAN_ILE_1	0x2

#define CAN_MES_RAM_BASE_ADDR 0x2000

#define MTTCAN_RAM_SIZE_IN_WORDS      ((4096) >> 2)
#define RAM_WORD_WIDTH_IN_BYTES		4

#define NBITRATE_PRESCALAR_500K 0x4409
#define NBITRATE_PRESCALAR_1M   0x1c09
#define DBITRATE_PRESCALAR_2M   0x0d40

/* Message RAM - Partitioning */
#define RAM_WORDS_PER_MTT_CAN  1024
#define RAM_BYTES_PER_MTT_CAN  (RAM_WORDS_PER_MTT_CAN * RAM_WORD_WIDTH_IN_BYTES)

#define MAX_11_BIT_FILTER_ELEMS 128
#define MAX_29_BIT_FILTER_ELEMS  64
#define MAX_RX_FIFO_0_ELEMS      32
#define MAX_RX_FIFO_1_ELEMS      32
#define MAX_RX_BUFFER_ELEMS      64
#define MAX_TX_EVENT_FIFO_ELEMS  32
#define MAX_TX_BUFFER_ELEMS      32
#define MAX_TRIG_ELEMS           64

/* Element Size in Words */
#define MAX_RX_BUF_ELEM_SIZE_WORD     18
#define MAX_TX_BUF_ELEM_SIZE_WORD     18
#define TX_EVENT_FIFO_ELEM_SIZE_WORD   2
#define STD_ID_FILTER_ELEM_SIZE_WORD   1
#define EXT_ID_FILTER_ELEM_SIZE_WORD   2

#define TX_BUF_ELEM_HEADER_WORD        2
#define RX_BUF_ELEM_HEADER_WORD        2

/* Message RAM config (in Byte) */
/*	+-------------------+
 *	|   11 bit filter   | 16 * 01 * 4
 *	+-------------------+ = 64
 *	|   29 bit filter   | 16 * 02 * 4
 *	+-------------------+ = 128 + 64
 *	|   RX FIFO 0       | 8 * 18 * 4
 *	+-------------------+ = 576 + 192
 *	|   RX FIFO 1       | 8 * 18 * 4
 *	+-------------------+ = 576 + 768
 *	|   RX BUFFERS      | 8 * 18 * 4
 *	+-------------------+ = 576 + 1344
 *	|   TX EVENT FIFO   | 16 * 02 * 4
 *	+-------------------+ = 128 + 1920
 *	|   TX BUFFERS      | 16 * 18 * 4
 *	+-------------------+ = 1152 + 2048
 *	|   MEM TRIGG ELMTS | 16 * 02 * 4
 *	+-------------------+ = 128 + 3232 = 3360
 */

#define CONF_11_BIT_FILTER_ELEMS 16
#define CONF_29_BIT_FILTER_ELEMS 16
#define CONF_RX_FIFO_0_ELEMS     8
#define CONF_RX_FIFO_1_ELEMS     8
#define CONF_RX_BUFFER_ELEMS     8
#define CONF_TX_EVENT_FIFO_ELEMS 16
#define CONF_TX_BUFFER_ELEMS     8
#define CONF_TX_FIFO_ELEMS       8
#define CONF_TRIG_ELEMS          16

#define MAX_RXB_ELEM_SIZE       72
#define MAX_TXB_ELEM_SIZE       72
#define TX_EVENT_FIFO_ELEM_SIZE 8
#define SIDF_ELEM_SIZE          4
#define XIDF_ELEM_SIZE          8
#define TXB_ELEM_HEADER_SIZE    8
#define RXB_ELEM_HEADER_SIZE    8
#define TRIG_ELEM_SIZE          8


#define REL_SIDFC_FLSSA  0
#define REL_XIDFC_FLESA (REL_SIDFC_FLSSA + (CONF_11_BIT_FILTER_ELEMS * SIDF_ELEM_SIZE))
#define REL_RXF0C_F0SA  (REL_XIDFC_FLESA + (CONF_29_BIT_FILTER_ELEMS * XIDF_ELEM_SIZE))
#define REL_RXF1C_F1SA  (REL_RXF0C_F0SA  + (CONF_RX_FIFO_0_ELEMS     * MAX_RXB_ELEM_SIZE))
#define REL_RXBC_RBSA   (REL_RXF1C_F1SA  + (CONF_RX_FIFO_1_ELEMS     * MAX_RXB_ELEM_SIZE))
#define REL_TXEFC_EFSA  (REL_RXBC_RBSA   + (CONF_RX_BUFFER_ELEMS     * MAX_RXB_ELEM_SIZE))
#define REL_TXBC_TBSA   (REL_TXEFC_EFSA  + (CONF_TX_EVENT_FIFO_ELEMS * TX_EVENT_FIFO_ELEM_SIZE))
#define REL_TMC_TMSA    (REL_TXBC_TBSA   + (CONF_TX_BUFFER_ELEMS     * MAX_TXB_ELEM_SIZE))
#define REL_TMC_END     (REL_TMC_TMSA    + (CONF_TRIG_ELEMS          * TRIG_ELEM_SIZE) - 1)

/* Last Error Code */
enum ttcan_lec_type {
	LEC_NO_ERROR = 0,
	LEC_STUFF_ERROR = 1,
	LEC_FORM_ERROR = 2,
	LEC_ACK_ERROR = 3,
	LEC_BIT1_ERROR = 4,
	LEC_BIT0_ERROR = 5,
	LEC_CRC_ERROR = 6,
	LEC_NO_CHANGE = 7,
};

enum ttcan_timestamp_source {
	TS_DISABLE = 0,
	TS_INTERNAL = 1,
	TS_EXTERNAL = 2,
	TS_DISABLE2 = 3
};

enum ttcan_rx_type {
	BUFFER = 1,
	FIFO_0 = 2,
	FIFO_1 = 4,
	TX_EVT = 8
};

static const uint8_t fdcan_dlc2len[] = {
	0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64 };

static const uint8_t fdcan_len2dlc[] = {
	0, 1, 2, 3, 4, 5, 6, 7, 8,	/* 0 - 8 */
	9, 9, 9, 9,			/* 9 - 12 */
	10, 10, 10, 10,			/* 13 - 16 */
	11, 11, 11, 11,			/* 17 - 20 */
	12, 12, 12, 12,			/* 21 - 24 */
	13, 13, 13, 13, 13, 13, 13, 13,	/* 25 - 32 */
	14, 14, 14, 14, 14, 14, 14, 14,	/* 33 - 40 */
	14, 14, 14, 14, 14, 14, 14, 14,	/* 41 - 48 */
	15, 15, 15, 15, 15, 15, 15, 15,	/* 49 - 56 */
	15, 15, 15, 15, 15, 15, 15, 15	/* 57 - 64 */
};

enum ttcan_mram_item {
	MRAM_SIDF = 0,
	MRAM_XIDF,
	MRAM_RXF0,
	MRAM_RXF1,
	MRAM_RXB,
	MRAM_TXE,
	MRAM_TXB,
	MRAM_TMC,
	MRAM_ELEMS
};

enum ttcan_tx_conf {
        TX_CONF_TXB = 0,
        TX_CONF_TXQ,
        TX_CONF_QMODE,
        TX_CONF_BSIZE,
        TX_CONF_MAX
};

enum ttcan_rx_conf {
	RX_CONF_RXB = 0,
	RX_CONF_RXF0,
	RX_CONF_RXF1,
	RX_CONF_MAX
};

struct ttcan_mram_elem {
	uint16_t off;
	uint16_t num;
};

static inline uint8_t ttcan_dlc2len(uint8_t dlc)
{
	return fdcan_dlc2len[dlc & 0x0F];
}

static inline uint8_t ttcan_len2dlc(uint8_t len)
{
	if (len > 64)
		return 0xF;
	return fdcan_len2dlc[len];
}

static inline enum ttcan_data_field_size
get_dfs(uint32_t bytes)
{
	switch (bytes) {
	case 8:
		return BYTE8;
	case 12:
		return BYTE12;
	case 16:
		return BYTE16;
	case 20:
		return BYTE20;
	case 24:
		return BYTE24;
	case 32:
		return BYTE32;
	case 48:
		return BYTE48;
	case 64:
		return BYTE64;
	default:
		return 0;
	}
}

static inline int data_in_element(
	enum ttcan_data_field_size dfs)
{
	switch (dfs) {
	case BYTE8:
		return 8;
	case BYTE12:
		return 12;
	case BYTE16:
		return 16;
	case BYTE20:
		return 20;
	case BYTE24:
		return 24;
	case BYTE32:
		return 32;
	case BYTE48:
		return 48;
	case BYTE64:
		return 64;
	default:
		return 0;
	}
}

static inline uint32_t ttcan_xread32(struct ttcan_controller *ttcan, int reg)
{
	return (uint32_t) readl(ttcan->xbase + reg);
}

static inline uint32_t ttcan_read32(struct ttcan_controller *ttcan, int reg)
{
	return (uint32_t) readl(ttcan->base + reg);
}

static inline void ttcan_xwrite32(struct ttcan_controller *ttcan, int reg, uint32_t val)
{
	writel(val, ttcan->xbase + reg);
}

static inline void ttcan_write32(struct ttcan_controller *ttcan, int reg, uint32_t val)
{
	writel(val, ttcan->base + reg);
}

static inline int ttcan_protected(uint32_t cccr_reg)
{
	if ((cccr_reg & 0x3) != 0x3) {
		error_hook("protected");
		return -EPERM;
	}
	return 0;
}

void ttcan_print_version(struct ttcan_controller *ttcan);
void ttcan_set_ok(struct ttcan_controller *ttcan);
int ttcan_set_init(struct ttcan_controller *ttcan);
int ttcan_reset_init(struct ttcan_controller *ttcan);
int ttcan_set_power(struct ttcan_controller *ttcan, int val);
int ttcan_set_config_change_enable(struct ttcan_controller *ttcan);
void ttcan_reset_config_change_enable(struct ttcan_controller *ttcan);

int ttcan_raw_read_rx_msg_ram(struct ttcan_controller *ttcan,
			  uint32_t addr_in_msg_ram,
			  uint32_t *ttcanfd);
int ttcan_read_rx_msg_ram(struct ttcan_controller *ttcan,
			  uint32_t addr_in_msg_ram,
			  struct canfd_frame *ttcanfd);
int ttcan_write_tx_msg_ram(struct ttcan_controller *ttcan,
			   uint32_t addr_in_msg_ram,
			   struct canfd_frame *ttcanfd,
			   uint8_t index);

unsigned int ttcan_read_txevt_fifo(struct ttcan_controller *ttcan,
		struct mttcanfd_frame *fd);

unsigned int ttcan_read_rx_fifo(struct ttcan_controller *ttcan);
unsigned int ttcan_read_rx_fifo0(struct ttcan_controller *ttcan);
unsigned int ttcan_read_rx_fifo1(struct ttcan_controller *ttcan);
unsigned int ttcan_read_hp_mesgs(struct ttcan_controller *ttcan,
					struct mttcanfd_frame *fd);
int mttcan_do_receive(struct ttcan_controller *ttcan,
		struct mttcanfd_frame *fd);

void ttcan_set_rx_buffers_elements(struct ttcan_controller *ttcan);
void ttcan_set_rx_buffer_addr(struct ttcan_controller *ttcan);
void ttcan_set_rx_fifo0(struct ttcan_controller *ttcan,
			int num_elems, int fifo_watermark);
void ttcan_set_rx_fifo1(struct ttcan_controller *ttcan,
			int num_elems, int fifo_watermark);
void ttcan_config_rx_data_elem_sizes(struct ttcan_controller *ttcan,
	enum ttcan_data_field_size rxbuf_dfs,
	enum ttcan_data_field_size rxfifo0_dfs,
	enum ttcan_data_field_size rxfifo1_dfs);

int ttcan_set_tx_buffer_addr(struct ttcan_controller *ttcan, int num_buffer,
	int num_queue, enum ttcan_data_field_size dfs, int mode);
int ttcan_tx_fifo_full(struct ttcan_controller *ttcan);
int ttcan_tx_fifo_queue_msg(struct ttcan_controller *ttcan,
			    struct canfd_frame *ttcanfd);
int ttcan_tx_fifo_get_free_element(struct ttcan_controller *ttcan);

int ttcan_tx_buf_req_pending(struct ttcan_controller *ttcan, uint8_t index);
void ttcan_tx_ded_msg_write(struct ttcan_controller *ttcan,
			    struct canfd_frame *ttcanfd,
			    uint8_t index, bool tt_en);
int ttcan_tx_msg_buffer_write(struct ttcan_controller *ttcan,
				struct canfd_frame *ttcanfd,
				bool tt_en);
void ttcan_tx_trigger_msg_transmit(struct ttcan_controller *ttcan, uint8_t index);

void ttcan_reset_std_id_filter(struct ttcan_controller *ttcan, uint32_t list_size);
void ttcan_set_std_id_filter(struct ttcan_controller *ttcan, int filter_index,
			     uint8_t sft, uint8_t sfec, uint32_t sfid1, uint32_t sfid2);
uint32_t ttcan_get_std_id_filter(struct ttcan_controller *ttcan, int idx);

void ttcan_reset_xtd_id_filter(struct ttcan_controller *ttcan, uint32_t list_size);
void ttcan_set_xtd_id_filter(struct ttcan_controller *ttcan, int filter_index,
			     uint8_t eft, uint8_t efec, uint32_t efid1, uint32_t efid2);
void ttcan_set_xtd_mask_add(struct ttcan_controller *ttcan, int extid_mask);
uint64_t ttcan_get_xtd_id_filter(struct ttcan_controller *ttcan, int idx);

void ttcan_set_std_id_filter_addr(struct ttcan_controller *ttcan,
				  uint32_t list_size);
void ttcan_set_xtd_id_filter_addr(struct ttcan_controller *ttcan,
				  uint32_t list_size);
int ttcan_set_gfc(struct ttcan_controller *ttcan, uint32_t regval);
uint32_t ttcan_get_gfc(struct ttcan_controller *ttcan);

int ttcan_set_xidam(struct ttcan_controller *ttcan, uint32_t regval);
uint32_t ttcan_get_xidam(struct ttcan_controller *ttcan);

void ttcan_set_time_stamp_conf(struct ttcan_controller *ttcan,
				uint16_t timer_prescalar,
				enum ttcan_timestamp_source time_type);
void ttcan_set_txevt_fifo_conf(struct ttcan_controller *ttcan,
				uint8_t water_mark, uint8_t size);
/* Mesg RAM partition */
int ttcan_mesg_ram_config(struct ttcan_controller *ttcan);
uint32_t ttcan_read_ecr(struct ttcan_controller *ttcan);
uint32_t ttcan_read_tx_complete_reg(struct ttcan_controller *ttcan);
void ttcan_set_tx_cancel_request(struct ttcan_controller *ttcan, uint32_t txbcr);
uint32_t ttcan_read_tx_cancelled_reg(struct ttcan_controller *ttcan);
uint32_t ttcan_read_psr(struct ttcan_controller *ttcan);
int ttcan_read_rx_buffer(struct ttcan_controller *ttcan);
int ttcan_set_bitrate(struct ttcan_controller *ttcan, uint32_t bitrate,
		uint32_t dbitrate);
int ttcan_read_txevt_ram(struct ttcan_controller *ttcan,
	uint32_t read_addr, struct mttcan_tx_evt_element *txevt);
int ttcan_tx_req_pending(struct ttcan_controller *ttcan);

void ttcan_disable_auto_retransmission(struct ttcan_controller *ttcan, bool enable);
int ttcan_set_bus_monitoring_mode(struct ttcan_controller *ttcan, bool enable);
int ttcan_set_loopback(struct ttcan_controller *ttcan);
int ttcan_set_normal_mode(struct ttcan_controller *ttcan);

/* Interrupt APIs */
void ttcan_clear_intr(struct ttcan_controller *ttcan);
void ttcan_clear_tt_intr(struct ttcan_controller *ttcan);
void ttcan_ir_write(struct ttcan_controller *ttcan, uint32_t value);
void ttcan_select_enable_intr(struct ttcan_controller *ttcan, uint32_t value);
void ttcan_ttir_write(struct ttcan_controller *ttcan, uint32_t value);
void ttcan_ttier_write(struct ttcan_controller *ttcan, uint32_t value);
uint32_t ttcan_read_ir(struct ttcan_controller *ttcan);
uint32_t ttcan_read_ttir(struct ttcan_controller *ttcan);

/* list APIs */
int tegra_add_msg_ctlr_list(struct ttcan_controller *ttcan, struct mttcanfd_frame *fd);
#endif
