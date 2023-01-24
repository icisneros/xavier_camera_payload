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

#ifndef __SPE_CAN_H
#define __SPE_CAN_H

#include <FreeRTOS.h>
#include <queue.h>
#include <semphr.h>

#define MTT_MAX_DATA_LEN 64
#define MTT_MAX_DLC	15

/*Size of data in an element */
enum ttcan_data_field_size {
	BYTE8 = 0,
	BYTE12 = 1,
	BYTE16 = 2,
	BYTE20 = 3,
	BYTE24 = 4,
	BYTE32 = 5,
	BYTE48 = 6,
	BYTE64 = 7
};

/* bit 0 - 28 : CAN identifier
 * bit 29 : type of frame (0 = data, 1 = error)
 * bit 30 : RTR
 * bit 31 : frame format type (0 = std, 1 = ext)
 */

/* bit 0 : 1 = BRS; 0 = Normal
 * bit 1 : 1 = Error Passive; 0 = Error Active
 * bit 2 : 1 = CAN FD; 0 = Normal
 * bit 4 : 1 = RX; 0 = Tx Direction
 */


struct canfd_frame {
	uint32_t can_id;	/* FMT/RTR/ERR/ID */
	uint8_t d_len;		/* data length */
	uint8_t flags;		/* FD flags */
	uint8_t resv0;
	uint8_t resv1;
	uint8_t data[MTT_MAX_DATA_LEN];
	/* Any new structure entries should be placed below the comment */
	uint32_t tstamp;
};

/*Size of data in an element */
enum m_ttcan_msgid {
	MTTCAN_MSG_TX = 1,
	MTTCAN_MSG_RX = 2,
	MTTCAN_MSG_TX_COMPL = 3,
	MTTCAN_MSG_STAT_CHG = 4,
	MTTCAN_MSG_BERR_CHG = 5,
	MTTCAN_MSG_RX_LOST_FRAME = 6,
	MTTCAN_MSG_TXEVT = 7,
	MTTCAN_CMD_CAN_ENABLE = 8,
	MTTCAN_MSG_LAST
};

struct mttcanfd_frame {
	uint16_t cmdid;
	uint16_t ext_cmdid;
	union {
		struct canfd_frame frame;
		uint32_t data[19];
	} payload;
};

struct ttcan_msg_ram {
	uint32_t base;		/* physical address of the message ram base */
	uint32_t virt_base;
	uint32_t sidfc_flssa;
	uint32_t xidfc_flesa;
	uint32_t rxf0c_f0sa;
	uint32_t rxf1c_f1sa;
	uint32_t rxbc_rbsa;
	uint32_t txefc_efsa;
	uint32_t txbc_tbsa;
	uint32_t tmc_tmsa;
};

struct ttcan_element_size {
	uint16_t rx_fifo0;
	uint16_t rx_fifo1;
	uint16_t rx_buffer;
	uint16_t tx_buffer;
	uint16_t tx_fifo;
};

struct ttcan_txbuff_config {
	uint32_t fifo_q_num;
	uint32_t ded_buff_num;
	uint32_t evt_q_num;
	enum ttcan_data_field_size dfs;
	uint32_t flags;		/* bit 0: 0=Fifo, 1=Queue */
};

struct ttcan_rxbuff_config {
	uint32_t rxq0_size;
	uint32_t rxq1_size;
	uint32_t rxb_dsize;
	uint64_t rxq0_bmsk;
	uint64_t rxq1_bmsk;
	uint64_t rxb_bmsk;
};

struct ttcan_filter_config {
	uint32_t std_fltr_size;
	uint32_t xtd_fltr_size;
};

struct device_stats {
	uint32_t rx_packets;
	uint32_t tx_packets;
	uint32_t rx_bytes;
	uint32_t tx_bytes;
	uint32_t rx_errors;
	uint32_t rx_dropped;
	uint32_t rx_over_errors;
	uint32_t tx_aborted_errors;
	uint32_t tx_dropped;
};

struct ttcan_controller {
	uint32_t base;	/* controller regs space should be remapped. */
	uint32_t xbase;      /* extra registers are mapped */
	const struct tegra_rst *rst; /* CAR registers are mapped */
	size_t mram_base;
	uint32_t id;
	uint32_t irq;
	uint32_t proto_state;
	uint32_t ts_prescalar;
	uint32_t tt_mem_elements;
	uint32_t tx_object;
	uint32_t tx_obj_cancelled;
	uint64_t ts_counter;
	bool tx_full;
	BaseType_t higher_prio_task_woken;
	struct ttcan_msg_ram mram_sa;
	struct ttcan_element_size e_size;
	struct ttcan_txbuff_config tx_config;
	struct ttcan_rxbuff_config rx_config;
	struct ttcan_filter_config fltr_config;
	int buf_idx;
	int data;
	struct device_stats stats;
	uint8_t m_id[32];
	void *priv;
	QueueHandle_t isr_q;
	SemaphoreHandle_t can_tx_sem;
};

BaseType_t mttcan_controller_init(struct ttcan_controller *ttcan);
void mttcan_controller_deinit(struct ttcan_controller *ttcan);
int mttcan_controller_enable(struct ttcan_controller *ttcan, bool state);
int mttcan_transfer(struct ttcan_controller *ttcan, struct mttcanfd_frame *fd);
BaseType_t tegra_can_receive(struct ttcan_controller *ttcan,
	       struct mttcanfd_frame *cf, TickType_t timeout);
#endif
