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

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#include <macros.h>
#include <clk-tegra.h>
#include <tegra-mttcan.h>
#include <tegra-can.h>
#include <can-tegra-hw.h>
#include <irqs.h>
#include <irqs-hw.h>

#ifdef ENABLE_DEBUG_PRINT
#define dbgprintf_isr printf_isr
#define DEBUG
#else
static inline void dbgprintf_isr(const char *fmt, ...) {}
#endif

static void tegra_can_irq_handler(void *canptr);

#define MTTCAN_IE		0x3BBEF7FF
#define MTTCAN_RX_FIFO_INTR	(0xFF)
#define MTTCAN_RX_HP_INTR	(0x1 << 8)
#define MTTCAN_ERR_INTR		(0x1FF9 << 17)

#define CAN_STATE_ERROR_ACTIVE	0
#define CAN_STATE_ERROR_WARNING	1
#define CAN_STATE_ERROR_PASSIVE	2
#define CAN_STATE_BUS_OFF	3

#define CAN_ISR_Q_SIZE	4

static int tegra_mttcan_ctlr_init(struct ttcan_controller *ttcan)
{
	int err = 0;
	uint32_t gfc_reg = 0;
	uint32_t tx_mode = 0; /* FIFO mode */
	enum ttcan_data_field_size dfs = BYTE64;

	/* Reset CAR */
	tegra_clk_reset_pulse(ttcan->rst, 1);
	/* Enable Access */
	ttcan_set_ok(ttcan);
	/* Power UP */
	ttcan_set_power(ttcan, 1);

	err = ttcan_mesg_ram_config(ttcan);
	if (err)
		return err;

	err = ttcan_set_config_change_enable(ttcan);
	if (err)
		return err;

	/* Accept unmatched in Rx FIFO0 and accept all remote frame */
	gfc_reg = (GFC_ANFS_RXFIFO_0 | GFC_ANFE_RXFIFO_0);

	err = ttcan_set_gfc(ttcan, gfc_reg);
	if (err)
		return err;

	/* Reset XIDAM to default */
	ttcan_set_xidam(ttcan, DEF_MTTCAN_XIDAM);

	/* Set Rx buffers */
	ttcan_set_rx_buffer_addr(ttcan);
	ttcan_set_rx_fifo0(ttcan, CONF_RX_FIFO_0_ELEMS, (CONF_RX_FIFO_0_ELEMS/2));
	ttcan_set_rx_fifo1(ttcan, CONF_RX_FIFO_1_ELEMS, (CONF_RX_FIFO_1_ELEMS/2));
	ttcan_config_rx_data_elem_sizes(ttcan, dfs, dfs, dfs);

	ttcan_set_std_id_filter_addr(ttcan, CONF_11_BIT_FILTER_ELEMS);
	ttcan_set_xtd_id_filter_addr(ttcan, CONF_29_BIT_FILTER_ELEMS);

	ttcan_reset_std_id_filter(ttcan, CONF_11_BIT_FILTER_ELEMS);
	ttcan_reset_xtd_id_filter(ttcan, CONF_29_BIT_FILTER_ELEMS);

	/* Enable Time Stamp Prescalar*/
	ttcan_set_time_stamp_conf(ttcan, 9, TS_INTERNAL);

	/* Set Tx Event FIFO */
	ttcan_set_txevt_fifo_conf(ttcan, CONF_TX_EVENT_FIFO_ELEMS/2,  CONF_TX_EVENT_FIFO_ELEMS);

	/* Set Tx buffers */
	ttcan_set_tx_buffer_addr(ttcan, CONF_TX_BUFFER_ELEMS, CONF_TX_FIFO_ELEMS, dfs, tx_mode);

	ttcan_clear_intr(ttcan);
	ttcan_print_version(ttcan);

	return err;
}

BaseType_t mttcan_controller_init(struct ttcan_controller *ttcan)
{
	ttcan->isr_q = xQueueCreate(CAN_ISR_Q_SIZE, sizeof(struct mttcanfd_frame));
	if (ttcan->isr_q == NULL)
		return pdFAIL;

	ttcan->can_tx_sem = xSemaphoreCreateBinary();
	if (ttcan->can_tx_sem == NULL) {
		vQueueDelete(ttcan->isr_q);
		ttcan->isr_q = NULL;
		return pdFAIL;
	}

	tegra_mttcan_ctlr_init(ttcan);
	ttcan_set_bitrate(ttcan, NBITRATE_PRESCALAR_500K, DBITRATE_PRESCALAR_2M);
	ttcan_reset_init(ttcan);
	irq_set_handler(ttcan->irq, tegra_can_irq_handler, (void *)ttcan);
	return pdPASS;
}

void mttcan_controller_deinit(struct ttcan_controller *ttcan)
{
	ttcan_set_init(ttcan);

	if (ttcan->can_tx_sem) {
		vSemaphoreDelete(ttcan->can_tx_sem);
		ttcan->can_tx_sem = NULL;
	}

	if (ttcan->isr_q) {
		vQueueDelete(ttcan->isr_q);
		ttcan->isr_q = NULL;
	}
}

int mttcan_controller_enable(struct ttcan_controller *ttcan, bool state)
{
	int err = 0;

	if (state == true) {
		err = ttcan_set_config_change_enable(ttcan);
		if (err)
			return err;
		ttcan_clear_intr(ttcan);
		ttcan_select_enable_intr(ttcan, MTTCAN_IE);
		ttcan_reset_init(ttcan);
		irq_enable(ttcan->irq);
	}
	else {
		irq_disable(ttcan->irq);
		ttcan_select_enable_intr(ttcan, 0);
		err = ttcan_set_config_change_enable(ttcan);
		if (err)
			return err;
	}
	return err;
}

int tegra_add_msg_ctlr_list(struct ttcan_controller *ttcan, struct mttcanfd_frame *fd)
{
	if (ttcan->isr_q != 0) {
		if (xQueueSendToBackFromISR(ttcan->isr_q, fd,
			&ttcan->higher_prio_task_woken) == errQUEUE_FULL) {
			error_hook("Queue full");
			return -1;
		}
	}
	return 0;
}

BaseType_t tegra_can_receive(struct ttcan_controller *ttcan,
	       struct mttcanfd_frame *cf, TickType_t timeout)
{
	return xQueueReceive(ttcan->isr_q, cf, timeout);
}

static void mttcan_handle_lost_frame(struct ttcan_controller *ttcan, int fifo_num)
{
	uint32_t ack_ir;

	if (fifo_num)
		ack_ir = MTT_IR_RF1L_MASK;
	else
		ack_ir = MTT_IR_RF0L_MASK;
	ttcan_ir_write(ttcan, ack_ir);
}

int mttcan_do_receive(struct ttcan_controller *ttcan,
		struct mttcanfd_frame *fd)
{
	if (tegra_add_msg_ctlr_list(ttcan, fd) < 0) {
		ttcan->stats.rx_dropped++;
		return -ENOMEM;
	}

	ttcan->stats.rx_packets++;
	ttcan->stats.rx_bytes += fd->payload.frame.d_len;
	return 1;
}

static void mttcan_tx_complete(struct ttcan_controller *ttcan,
		struct mttcanfd_frame *fd)
{
	uint32_t msg_no;
	uint32_t completed_tx = ttcan_read_tx_complete_reg(ttcan);
	uint32_t txbitmap;

	fd->cmdid = MTTCAN_MSG_TX_COMPL;
	fd->payload.data[0] = 0;

	txbitmap = ttcan->tx_object & completed_tx;

	/* Check if we get completion on full CAN Tx buffer */
	if (txbitmap && ttcan->tx_full) {
		ttcan->tx_full = false;
		xSemaphoreGiveFromISR(ttcan->can_tx_sem,
				&ttcan->higher_prio_task_woken);

	}

	while (txbitmap) {
		msg_no = ffs(txbitmap) - 1;
		txbitmap &= ~(1 << msg_no);
		/* Map the completed msgs to CCPlex IDs */
		ttcan->tx_object &= ~(1 << msg_no);
		fd->payload.data[0] |= (1 << ttcan->m_id[msg_no]);
		ttcan->stats.tx_packets++;
	}

	/* Return if there are no completed messages to process */
	if (!fd->payload.data[0]) {
		error_hook("No Tx completion ack received");
		return;
	}

	/* Acknowledged completed messages to CCPlex */
	if (tegra_add_msg_ctlr_list(ttcan, fd) < 0)
		error_hook("Failed to send message to CCPlex");

	return;
}

static void mttcan_tx_cancelled(struct ttcan_controller *ttcan)
{
	uint32_t msg_no;
	uint32_t cancelled_msg;

	msg_no = ttcan_read_tx_cancelled_reg(ttcan);

	/* Extract updated cancelled message bits for current interrupt */
	cancelled_msg = (ttcan->tx_obj_cancelled ^ msg_no) &
			~(ttcan->tx_obj_cancelled);
	cancelled_msg &= ~ttcan->tx_object;
	ttcan->tx_obj_cancelled = msg_no;

	/* Check if we get completion on full CAN Tx buffer */
	if (cancelled_msg && ttcan->tx_full) {
		ttcan->tx_full = false;
		xSemaphoreGiveFromISR(ttcan->can_tx_sem,
				&ttcan->higher_prio_task_woken);

	}

	while (cancelled_msg) {
		msg_no = ffs(cancelled_msg);
		cancelled_msg &= ~(1 << (msg_no - 1));
		ttcan->tx_object &= ~(1 << (msg_no - 1));
		ttcan->stats.tx_aborted_errors++;
	}
}

static void tegra_can_irq_handler(void *canptr)
{
	int rec_msgs = 0;
	uint32_t ir, ack, ttir, ttack, psr;
	struct mttcanfd_frame fd;
	struct ttcan_controller *ttcan = (struct ttcan_controller *)canptr;

	ttcan->higher_prio_task_woken = pdFALSE;
	ttir = ttcan_read_ttir(ttcan);
	ir = ttcan_read_ir(ttcan);
	if (!ir)
		return;

	if (ir & MTTCAN_ERR_INTR) {
		psr = ttcan_read_psr(ttcan);
		ttcan->proto_state = psr;
		ack = ir & MTTCAN_ERR_INTR;
		ttcan_ir_write(ttcan, ack);
		if ((ir & MTT_IR_EW_MASK) && (psr & MTT_PSR_EW_MASK)) {
			fd.cmdid = MTTCAN_MSG_STAT_CHG;
			fd.payload.data[0] = CAN_STATE_ERROR_WARNING;
			fd.payload.data[1] = ttcan_read_ecr(ttcan);
			if (tegra_add_msg_ctlr_list(ttcan, &fd) < 0)
				error_hook("Failed to notify state change");
			ttcan->stats.rx_packets++;
			ttcan->stats.rx_bytes += CAN_ERR_DLC_DLEN;
			warning_hook("entered error warning state");
		}
		if ((ir & MTT_IR_EP_MASK) && (psr & MTT_PSR_EP_MASK)) {
			fd.cmdid = MTTCAN_MSG_STAT_CHG;
			fd.payload.data[0] = CAN_STATE_ERROR_PASSIVE;
			fd.payload.data[1] = ttcan_read_ecr(ttcan);
			if (tegra_add_msg_ctlr_list(ttcan, &fd) < 0)
				error_hook("Failed to notify state change");
			ttcan->stats.rx_packets++;
			ttcan->stats.rx_bytes += CAN_ERR_DLC_DLEN;
			error_hook("entered error passive state");
		}
		if ((ir & MTT_IR_BO_MASK) && (psr & MTT_PSR_BO_MASK)) {
			fd.cmdid = MTTCAN_MSG_STAT_CHG;
			fd.payload.data[0] = CAN_STATE_BUS_OFF;
			fd.payload.data[1] = ttcan_read_ecr(ttcan);
			if (tegra_add_msg_ctlr_list(ttcan, &fd) < 0)
				error_hook("Failed to notify state change");
			ttcan->stats.rx_packets++;
			ttcan->stats.rx_bytes += CAN_ERR_DLC_DLEN;
			error_hook("entered bus off state");
		}
		if (((ir & MTT_IR_EP_MASK) && !(psr & MTT_PSR_EP_MASK))
			|| ((ir & MTT_IR_EW_MASK) && !(psr & MTT_PSR_EW_MASK))) {
			if (ir & MTT_IR_EP_MASK)
				dbgprintf_isr("left error passive state\r\n");
			else
				dbgprintf_isr("left error warning state\r\n");
			fd.cmdid = MTTCAN_MSG_STAT_CHG;
			fd.payload.data[0] = CAN_STATE_ERROR_ACTIVE;
			fd.payload.data[1] = ttcan_read_ecr(ttcan);
			if (tegra_add_msg_ctlr_list(ttcan, &fd) < 0)
				error_hook("Failed to notify state change");
		}

		/* Handle Bus error change */
		if ((ir & MTT_IR_PED_MASK) || (ir & MTT_IR_PEA_MASK)) {
			enum ttcan_lec_type lec;

			if (ir & MTT_IR_PEA_MASK)
				lec = (psr & MTT_PSR_LEC_MASK) >> MTT_PSR_LEC_SHIFT;
			else
				lec = (psr & MTT_PSR_DLEC_MASK) >> MTT_PSR_DLEC_SHIFT;

			fd.cmdid = MTTCAN_MSG_BERR_CHG;
			fd.payload.data[0] = lec;
			if (tegra_add_msg_ctlr_list(ttcan, &fd) < 0)
				error_hook("Failed to notify state change");
			ttcan->stats.rx_errors++;
			ttcan->stats.rx_packets++;
			ttcan->stats.rx_bytes += CAN_ERR_DLC_DLEN;

			error_hookf("IR = 0x%x PSR 0x%x", (unsigned)ir, (unsigned)psr);
		}
		if (ir & MTT_IR_WDI_MASK)
			warning_hook("Message RAM watchdog not handled");
	}

	if (ir & MTT_IR_TOO_MASK) {
		ack = MTT_IR_TOO_MASK;
		ttcan_ir_write(ttcan, ack);
		warning_hook("Rx timeout not handled");
	}

	/* High Priority Message */
	if (ir & MTTCAN_RX_HP_INTR) {
		ack = MTT_IR_HPM_MASK;
		ttcan_ir_write(ttcan, ack);
		if (ttcan_read_hp_mesgs(ttcan, &fd)) {
			dbgprintf_isr("%s: hp mesg received\r\n", __func__);
			if (mttcan_do_receive(ttcan, &fd) < 0)
				error_hook("Failed to send message");
		}
	}

	/* Handle dedicated buffer */
	if (ir & MTT_IR_DRX_MASK) {
		ack = MTT_IR_DRX_MASK;
		ttcan_ir_write(ttcan, ack);
		rec_msgs = ttcan_read_rx_buffer(ttcan);
		dbgprintf_isr("%s: buffer mesg received\r\n", __func__);
	}

	/* Handle RX Fifo interrupt */
	if (ir & MTTCAN_RX_FIFO_INTR) {
		if (ir & MTT_IR_RF1L_MASK) {
			ack = MTT_IR_RF1L_MASK;
			ttcan_ir_write(ttcan, ack);
			mttcan_handle_lost_frame(ttcan, 1);
			warning_hook("some msgs lost in Q1");

			ttcan->stats.rx_errors++;
			ttcan->stats.rx_over_errors++;
			fd.cmdid = MTTCAN_MSG_RX_LOST_FRAME;
			fd.payload.data[0] = 1;
			if (tegra_add_msg_ctlr_list(ttcan, &fd) < 0) {
				ttcan->stats.rx_dropped++;
				error_hook("Failed to send message");
			}
		}

		if (ir & MTT_IR_RF0L_MASK) {
			ack = MTT_IR_RF0L_MASK;
			ttcan_ir_write(ttcan, ack);
			mttcan_handle_lost_frame(ttcan, 0);
			warning_hook("some msgs lost in Q0");

			ttcan->stats.rx_errors++;
			ttcan->stats.rx_over_errors++;
			fd.cmdid = MTTCAN_MSG_RX_LOST_FRAME;
			fd.payload.data[0] = 0;
			if (tegra_add_msg_ctlr_list(ttcan, &fd) < 0) {
				ttcan->stats.rx_dropped++;
				error_hook("Failed to send message");
			}
		}

		if (ir & (MTT_IR_RF1F_MASK | MTT_IR_RF1W_MASK |
					MTT_IR_RF1N_MASK)) {
			ack = ir & (MTT_IR_RF1F_MASK |
					MTT_IR_RF1W_MASK |
					MTT_IR_RF1N_MASK);
			ttcan_ir_write(ttcan, ack);

			rec_msgs = ttcan_read_rx_fifo1(ttcan);
			dbgprintf_isr("%s: %d msg received in Q1\r\n", __func__, rec_msgs);
		}

		if (ir & (MTT_IR_RF0F_MASK | MTT_IR_RF0W_MASK |
					MTT_IR_RF0N_MASK)) {
			ack = ir & (MTT_IR_RF0F_MASK |
					MTT_IR_RF0W_MASK |
					MTT_IR_RF0N_MASK);
			ttcan_ir_write(ttcan, ack);
			rec_msgs = ttcan_read_rx_fifo0(ttcan);
			dbgprintf_isr("%s: %d msg received in Q0\r\n", __func__, rec_msgs);
		}
	}

	/* Handle Timer wrap around */
	if (ir & MTT_IR_TSW_MASK) {
		ack = MTT_IR_TSW_MASK;
		ttcan_ir_write(ttcan, ack);
	}

	/* Handle Transmission cancellation finished
	 * TCF interrupt is set when transmission cancelled is request
	 * by TXBCR register but in case wherer DAR (one-shot) is set
	 * the Tx buffers which transmission is not complete due to some
	 * reason are not retransmitted and for those buffers
	 * corresponding bit in TXBCF is set. Handle them to release
	 * Tx queue lockup in software.
	 */
	if ((ir & MTT_IR_TCF_MASK)) {
		if (ir & MTT_IR_TCF_MASK) {
			ack = MTT_IR_TCF_MASK;
			ttcan_ir_write(ttcan, ack);
		}
		mttcan_tx_cancelled(ttcan);
	}

	if (ir & MTT_IR_TC_MASK) {
		ack = MTT_IR_TC_MASK;
		ttcan_ir_write(ttcan, ack);
		mttcan_tx_complete(ttcan, &fd);
	}

	if (ir & MTT_IR_TFE_MASK) {
		/*
		 * netdev_info(dev, "Tx Fifo Empty %x\r\n", ir);
		 */
		ack = MTT_IR_TFE_MASK;
		ttcan_ir_write(ttcan, ack);
	}

	/* Handle Tx Event */
	if (ir & MTTCAN_TX_EV_FIFO_INTR) {
		/* New Tx Event */
		if ((ir & MTT_IR_TEFN_MASK) ||
				(ir & MTT_IR_TEFW_MASK)) {
			ttcan_read_txevt_fifo(ttcan, &fd);
		}

		if ((ir & MTT_IR_TEFL_MASK))
			warning_hook("Tx event lost\r\n");

		ack = MTTCAN_TX_EV_FIFO_INTR;
		ttcan_ir_write(ttcan, ack);
	}

	if (ttir) {
		ttack = 0xFFFFFFFF;
		ttcan_ttir_write(ttcan, ttack);
	}
//	portYIELD_FROM_ISR(ttcan->higher_prio_task_woken);
}

static int mttcan_start_xmit(struct ttcan_controller *ttcan, struct mttcanfd_frame *fd)
{
	int msg_no = -1;
	struct canfd_frame *cf = &fd->payload.frame;

	msg_no = ttcan_tx_msg_buffer_write(ttcan, cf, 0);

	if (msg_no < 0)
		msg_no = ttcan_tx_fifo_queue_msg(ttcan, cf);

	if (msg_no < 0) {
		ttcan->stats.tx_dropped++;
		return -ENOMEM;
	}

	taskENTER_CRITICAL();
	ttcan->tx_object |= (1 << msg_no);
	ttcan->tx_full = true;
	ttcan->tx_obj_cancelled &= ~(1 << msg_no);
	ttcan->m_id[msg_no] = fd->ext_cmdid;
	taskEXIT_CRITICAL();

	ttcan_tx_trigger_msg_transmit(ttcan, msg_no);

	return msg_no;
}

int mttcan_transfer(struct ttcan_controller *ttcan, struct mttcanfd_frame *fd)
{
	/* Retry till write to CAN message RAM is successful */
	while (mttcan_start_xmit(ttcan, fd) < 0)
		xSemaphoreTake(ttcan->can_tx_sem, portMAX_DELAY);

	return pdTRUE;
}
