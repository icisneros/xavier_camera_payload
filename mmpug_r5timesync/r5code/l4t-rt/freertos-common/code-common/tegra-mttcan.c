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

/**********************************************/
/********TTCAN Configurations functions *******/
/**********************************************/

#include <string.h>

#include <delay.h>
#include <tegra-mttcan.h>
#include <macros.h>

#define MTTCAN_INIT_TIMEOUT 1000

#ifdef ENABLE_DEBUG_PRINT
#define dbgprintf printf
#define DEBUG
#else
static inline void dbgprintf(const char *fmt, ...) {}
#endif

void ttcan_print_version(struct ttcan_controller *ttcan)
{
	uint32_t crel, endn;

	crel = ttcan_read32(ttcan, ADR_MTTCAN_CREL);
	endn = ttcan_read32(ttcan, ADR_MTTCAN_ENDN);

	dbgprintf("Release %d.%d.%d from %2.2x.%2.2x.201%1.1x\r\n",
		(crel & MTT_CREL_REL_MASK) >> MTT_CREL_REL_SHIFT,
		(crel & MTT_CREL_STEP_MASK) >> MTT_CREL_STEP_SHIFT,
		(crel & MTT_CREL_SUBS_MASK) >> MTT_CREL_SUBS_SHIFT,
		(crel & MTT_CREL_DAY_MASK) >> MTT_CREL_DAY_SHIFT,
		(crel & MTT_CREL_MON_MASK) >> MTT_CREL_MON_SHIFT,
		(crel & MTT_CREL_YEAR_MASK) >> MTT_CREL_YEAR_SHIFT);
	dbgprintf("CAN register access %s Endian Reg 0x%x\r\n",
		(endn == 0x87654321) ? "PASS" : "FAIL", endn);
}

static int ttcan_write32_check(struct ttcan_controller *ttcan,
		int reg, uint32_t val, uint32_t mask)
{
	uint32_t ret_val;

	ttcan_write32(ttcan, reg, val);

	ret_val = ttcan_read32(ttcan, reg);

	if ((ret_val & mask) == (val & mask))
		return 0;
	else
		error_hookf("%s failed, addr: 0x%x write 0x%x read 0x%x mask 0x%x",
			__func__, reg, (unsigned)val, (unsigned)ret_val, (unsigned)mask);
	return -EIO;
}

inline void ttcan_set_ok(struct ttcan_controller *ttcan)
{
	uint32_t val;

	val = ttcan_xread32(ttcan, ADDR_M_TTCAN_CNTRL_REG);
	val |= M_TTCAN_CNTRL_REG_COK;
	ttcan_xwrite32(ttcan, ADDR_M_TTCAN_CNTRL_REG, val);
}

int ttcan_set_init(struct ttcan_controller *ttcan)
{
	uint32_t cccr_reg;
	int timeout = MTTCAN_INIT_TIMEOUT;
	unsigned int ccr_reg = 0;

	cccr_reg = ttcan_read32(ttcan, ADR_MTTCAN_CCCR);

	if ((cccr_reg & MTT_CCCR_INIT_MASK) == 0) {
		/* Controller not yet initialized */
		cccr_reg |= 1;

		ttcan_write32(ttcan, ADR_MTTCAN_CCCR, cccr_reg);
		do {
			ccr_reg = ttcan_read32(ttcan, ADR_MTTCAN_CCCR);
			udelay(1);
			timeout--;
		} while ((((ccr_reg & MTT_CCCR_INIT_MASK) >>
				MTT_CCCR_INIT_SHIFT) != 1) && timeout);
		if (!timeout) {
			error_hook("Timeout");
			return -ETIMEDOUT;
		}
	}
	return 0;
}

int ttcan_reset_init(struct ttcan_controller *ttcan)
{
	uint32_t cccr_reg;
	int timeout = MTTCAN_INIT_TIMEOUT;
	unsigned int ccr_reg = 0;

	cccr_reg = ttcan_read32(ttcan, ADR_MTTCAN_CCCR);

	if (cccr_reg & MTT_CCCR_INIT_MASK) {
		/* Controller was initialized */
		cccr_reg &= ~1;

		ttcan_write32(ttcan, ADR_MTTCAN_CCCR, cccr_reg);
		do {
			ccr_reg = ttcan_read32(ttcan, ADR_MTTCAN_CCCR);
			udelay(1);
			timeout--;
		} while ((((ccr_reg & MTT_CCCR_INIT_MASK) >>
				MTT_CCCR_INIT_SHIFT) != 0) && timeout);
		if (!timeout) {
			error_hook("Timeout");
			return -ETIMEDOUT;
		}
	}
	return 0;
}

int ttcan_set_config_change_enable(struct ttcan_controller *ttcan)
{
	uint32_t cccr_reg;
	int timeout = MTTCAN_INIT_TIMEOUT;
	unsigned int ccr_reg = 0;

	/* initialize the core */
	if (ttcan_set_init(ttcan))
		return -ETIMEDOUT;

	/* set configuration change enable bit */
	cccr_reg = ttcan_read32(ttcan, ADR_MTTCAN_CCCR);
	cccr_reg |= MTT_CCCR_CCE_MASK;
	ttcan_write32(ttcan, ADR_MTTCAN_CCCR, cccr_reg);
	do {
		ccr_reg = ttcan_read32(ttcan, ADR_MTTCAN_CCCR);
		udelay(1);
		timeout--;
	} while ((((ccr_reg & MTT_CCCR_CCE_MASK) >>
			MTT_CCCR_CCE_SHIFT) != 1) && timeout);
	if (!timeout) {
		error_hook("Timeout");
		return -ETIMEDOUT;
	}

	return 0;
}

int ttcan_set_power(struct ttcan_controller *ttcan, int value)
{
	uint32_t cccr_reg;
	int timeout = MTTCAN_INIT_TIMEOUT;
	unsigned int ccr_reg = 0;

	cccr_reg = ttcan_read32(ttcan, ADR_MTTCAN_CCCR);
	cccr_reg &= ~(MTT_CCCR_CSR_MASK);
	cccr_reg |= ((!value) << MTT_CCCR_CSR_SHIFT) & MTT_CCCR_CSR_MASK;
	ttcan_write32(ttcan, ADR_MTTCAN_CCCR, cccr_reg);
	do {
		ccr_reg = ttcan_read32(ttcan, ADR_MTTCAN_CCCR);
		udelay(1);
		timeout--;
	} while ((((ccr_reg & MTT_CCCR_CSA_MASK) >>
			MTT_CCCR_CSA_SHIFT) != (!value)) && timeout);
	if (!timeout) {
		error_hook("Timeout");
		return -ETIMEDOUT;
	}

	return 0;
}

void ttcan_reset_config_change_enable(struct ttcan_controller *ttcan)
{
	/* reset the core */
	if (ttcan_reset_init(ttcan))
		error_hook("failed");

	/*CCCR.CCE is automatically reset when CCCR.INIT is reset */
}

void ttcan_disable_auto_retransmission(struct ttcan_controller *ttcan,
		bool enable)
{
	uint32_t cccr_reg;

	/* set DAR bit */
	cccr_reg = ttcan_read32(ttcan, ADR_MTTCAN_CCCR);
	if (enable)
		cccr_reg |= MTT_CCCR_DAR_MASK;
	else
		cccr_reg &= ~MTT_CCCR_DAR_MASK;
	ttcan_write32_check(ttcan, ADR_MTTCAN_CCCR, cccr_reg, MTTCAN_CCCR_MSK);
}

int ttcan_set_loopback(struct ttcan_controller *ttcan)
{
	uint32_t test_reg;
	uint32_t cccr_reg;
	int timeout = MTTCAN_INIT_TIMEOUT;
	unsigned int ccr_reg = 0;

	/* set TEST.LBCK (external loopback) bit */
	cccr_reg = ttcan_read32(ttcan, ADR_MTTCAN_CCCR);
	test_reg = ttcan_read32(ttcan, ADR_MTTCAN_TEST);

	if (ttcan_protected(cccr_reg))
		return -EPERM;

	cccr_reg |= MTT_CCCR_TEST_MASK;
	test_reg |= MTT_TEST_LBCK_MASK;

	ttcan_write32(ttcan, ADR_MTTCAN_CCCR, cccr_reg);
	do {
		ccr_reg = ttcan_read32(ttcan, ADR_MTTCAN_CCCR);
		udelay(1);
		timeout--;
	} while ((((ccr_reg & MTT_CCCR_TEST_MASK) >>
			MTT_CCCR_TEST_SHIFT) != 1) && timeout);
	if (!timeout) {
		error_hook("Timeout");
		return -ETIMEDOUT;
	}

	return ttcan_write32_check(ttcan, ADR_MTTCAN_TEST,
			test_reg, MTTCAN_TEST_MSK);

}

int ttcan_set_bus_monitoring_mode(struct ttcan_controller *ttcan, bool enable)
{
	uint32_t cccr_reg;

	/* set MON bit(bus monitor mode */
	cccr_reg = ttcan_read32(ttcan, ADR_MTTCAN_CCCR);
	if (ttcan_protected(cccr_reg))
		return -EPERM;
	if (enable)
		cccr_reg |= MTT_CCCR_MON_MASK;
	else
		cccr_reg &= ~MTT_CCCR_MON_MASK;
	return ttcan_write32_check(ttcan, ADR_MTTCAN_CCCR,
			cccr_reg, MTTCAN_CCCR_MSK);
}

int ttcan_set_normal_mode(struct ttcan_controller *ttcan)
{
	uint32_t cccr_reg;
	uint32_t test_reg;
	int timeout = MTTCAN_INIT_TIMEOUT;
	unsigned int ccr_reg = 0;

	/* Clear loopback and monitor mode */
	cccr_reg = ttcan_read32(ttcan, ADR_MTTCAN_CCCR);
	test_reg = ttcan_read32(ttcan, ADR_MTTCAN_TEST);

	if (ttcan_protected(cccr_reg))
		return -EPERM;

	if (test_reg & MTT_TEST_LBCK_MASK) {
		test_reg &= ~(MTT_TEST_LBCK_MASK);
		if ((cccr_reg & MTT_CCCR_TEST_MASK) == 0) {
			cccr_reg |= MTT_CCCR_TEST_MASK;
			ttcan_write32(ttcan, ADR_MTTCAN_CCCR, cccr_reg);
			do {
				ccr_reg = ttcan_read32(ttcan, ADR_MTTCAN_CCCR);
				udelay(1);
				timeout--;
			} while ((((ccr_reg & MTT_CCCR_TEST_MASK) >>
					MTT_CCCR_TEST_SHIFT) != 1) && timeout);
			if (!timeout) {
				error_hook("Timeout");
				return -ETIMEDOUT;
			}
		}
		ttcan_write32_check(ttcan, ADR_MTTCAN_TEST, test_reg,
				MTTCAN_TEST_MSK);
	}
	cccr_reg &= ~(MTT_CCCR_MON_MASK);
	cccr_reg &= ~(MTT_CCCR_TEST_MASK);
	return ttcan_write32_check(ttcan, ADR_MTTCAN_CCCR, cccr_reg,
			MTTCAN_CCCR_MSK);
}

inline uint32_t ttcan_read_ecr(struct ttcan_controller *ttcan)
{
	return ttcan_read32(ttcan, ADR_MTTCAN_ECR);
}

int ttcan_set_bitrate(struct ttcan_controller *ttcan, uint32_t bitrate,
		uint32_t dbitrate)
{
	int ret = 0;
	uint32_t tdcr_reg;
	uint32_t cccr_reg;

	ret = ttcan_write32_check(ttcan, ADR_MTTCAN_NBTP, bitrate,
			MTTCAN_NBTP_MSK);
	if (ret){
		error_hook("Normal bitrate configuration failed");
		return ret;
	}

	ret = ttcan_write32_check(ttcan, ADR_MTTCAN_DBTP, dbitrate,
			MTTCAN_DBTP_MSK);
	if (ret){
		error_hook("Fast bitrate configuration failed");
		return ret;
	}

	tdcr_reg = ttcan_read32(ttcan, ADR_MTTCAN_TDCR);
	tdcr_reg = (0 << MTT_TDCR_TDCO_SHIFT) & MTT_TDCR_TDCO_MASK;
	ttcan_write32(ttcan, ADR_MTTCAN_TDCR, tdcr_reg);

	cccr_reg = ttcan_read32(ttcan, ADR_MTTCAN_CCCR);
	cccr_reg |= MTT_CCCR_FDOE_MASK;
	cccr_reg |= MTT_CCCR_BRSE_MASK;
	ttcan_write32(ttcan, ADR_MTTCAN_CCCR, cccr_reg);

	return ret;
}

int ttcan_tx_req_pending(struct ttcan_controller *ttcan)
{
	uint32_t txbrp_reg = ttcan_read32(ttcan, ADR_MTTCAN_TXBRP);

	if (txbrp_reg)
		return 1;
	return 0;
}

int ttcan_raw_read_rx_msg_ram(struct ttcan_controller *ttcan, uint32_t read_addrs,
		uint32_t *d)
{
	int i = 2;
	uint32_t addr_in_msg_ram = read_addrs + ttcan->mram_base;
	int dlc = CANFD_MAX_DLEN;
	int words_to_read;

	d[0] = *(uint32_t *)(addr_in_msg_ram + (0 * CAN_WORD_IN_BYTES));
	d[1] = *(uint32_t *)(addr_in_msg_ram + (1 * CAN_WORD_IN_BYTES));
	dlc = ttcan_dlc2len((d[1] & RX_BUF_DLC_MASK) >> RX_BUF_DLC_SHIFT);
	words_to_read = (dlc + sizeof(uint32_t) - 1) / sizeof(uint32_t);

	for (i = 2; words_to_read > 0; i++) {
		d[i] = *(uint32_t *)(addr_in_msg_ram + (i * CAN_WORD_IN_BYTES));
		words_to_read--;
	}
	return 0;
}

int ttcan_read_txevt_ram(struct ttcan_controller *ttcan, uint32_t read_addr,
		struct mttcan_tx_evt_element *txevt)
{
	uint32_t msg_addr = read_addr + ttcan->mram_base;
	if (!txevt)
		return -1;

	txevt->f0 = readl(msg_addr);
	txevt->f1 = readl(msg_addr + CAN_WORD_IN_BYTES);

	return 0;
}

int ttcan_read_rx_msg_ram(struct ttcan_controller *ttcan, uint32_t read_addrs,
		struct canfd_frame *ttcanfd)
{
	int i = 0;
	uint32_t msg_data;
	uint32_t addr_in_msg_ram = read_addrs;

	if (!ttcanfd)
		return -1;

	ttcanfd->flags |= CAN_DIR_RX;

	ttcanfd->can_id = 0;
	ttcanfd->flags = 0;

	/* Update CAN ID and Flags from Rx msg */
	msg_data = *(uint32_t *)addr_in_msg_ram;
	if (msg_data & RX_BUF_XTD)
		ttcanfd->can_id = CAN_FMT | (msg_data & RX_BUF_EXTID_MASK);
	else
		ttcanfd->can_id =
			((msg_data & RX_BUF_STDID_MASK) >> RX_BUF_STDID_SHIFT);

	if (msg_data & RX_BUF_RTR)
		ttcanfd->can_id |= CAN_RTR;

	if (msg_data & RX_BUF_ESI)
		ttcanfd->flags |= CAN_ESI_FLAG;

	/* Update Flags and DLC from Rx msg */
	msg_data = *(uint32_t *)(addr_in_msg_ram + CAN_WORD_IN_BYTES);
	if (msg_data & RX_BUF_FDF)
		ttcanfd->flags |= CAN_FD_FLAG;

	if (msg_data & RX_BUF_BRS)
		ttcanfd->flags |= CAN_BRS_FLAG;

	ttcanfd->d_len = ttcan_dlc2len((msg_data & RX_BUF_DLC_MASK)
				>> RX_BUF_DLC_SHIFT);

	addr_in_msg_ram += 2 * CAN_WORD_IN_BYTES;

	/* Copy CAN payload */
	if (ttcanfd->d_len)
		memcpy(ttcanfd->data, (void *)addr_in_msg_ram, ttcanfd->d_len);

	dbgprintf("%s:received ID(0x%x) %s %s(%s)\r\n", __func__,
			(ttcanfd->can_id & CAN_FMT) ?
			(ttcanfd->can_id & CAN_EXT_ID_MASK) :
			(ttcanfd->can_id & CAN_STD_ID_MASK),
			(ttcanfd->can_id & CAN_FMT) ? "XTD" : "STD",
			(ttcanfd->flags & CAN_FD_FLAG) ? "FD" : "NON-FD",
			(ttcanfd->flags & CAN_BRS_FLAG) ? "BRS" : "NOBRS");
	return i;
}

int ttcan_write_tx_msg_ram(struct ttcan_controller *ttcan, uint32_t write_addrs,
		struct canfd_frame *ttcanfd, uint8_t index)
{
	uint32_t msg_data, idx;
	int bytes_to_write;
	uint32_t addr_in_msg_ram = write_addrs + ttcan->mram_base;
	addr_in_msg_ram += ttcan->mram_sa.virt_base - ttcan->mram_sa.base;

	/* T0 */
	if (ttcanfd->can_id & CAN_FMT)
		msg_data = (ttcanfd->can_id & CAN_EXT_ID_MASK) | TX_BUF_XTD;
	else
		msg_data =
			(ttcanfd->can_id & CAN_STD_ID_MASK) << TX_BUF_STDID_SHIFT;

	if (ttcanfd->can_id & CAN_RTR)
		msg_data |= TX_BUF_RTR;

	/*  This flag is ORed with error passive flag while sending */
	if (ttcanfd->flags & CAN_ESI_FLAG)
		msg_data |= TX_BUF_ESI;

	/* dbgprintf("T0: addr %x msg %x\n", addr_in_msg_ram, msg_data); */
	writel(msg_data, addr_in_msg_ram);

	/* T1 */
	msg_data =
		(ttcan_len2dlc(ttcanfd->d_len) << TX_BUF_DLC_SHIFT) &
		TX_BUF_DLC_MASK;

	if (ttcan->tx_config.evt_q_num)
		msg_data |= TX_BUF_EFC;

	if (ttcanfd->flags & CAN_FD_FLAG)
		msg_data |= TX_BUF_FDF;

	if (ttcanfd->flags & CAN_BRS_FLAG)
		msg_data |= TX_BUF_BRS;

	msg_data |= index <<  TX_BUF_MM_SHIFT;

	dbgprintf("%s:buf_id(%d):- %s(%s)\r\n", __func__, index,
			(ttcanfd->flags & CAN_FD_FLAG) ? "FD" : "NON-FD",
			(ttcanfd->flags & CAN_BRS_FLAG) ? "BRS" : "NOBRS");

	dbgprintf("T1: addr %p msg %x\n",
			(addr_in_msg_ram + CAN_WORD_IN_BYTES), msg_data);

	writel(msg_data, (addr_in_msg_ram + CAN_WORD_IN_BYTES));

	bytes_to_write = ttcanfd->d_len;

	idx = 0;

	while (bytes_to_write > 0) {
		msg_data = 0;
		switch (bytes_to_write) {
		default:
		case 4:
			msg_data = ttcanfd->data[idx + 3] << 24;
		case 3:
			msg_data += ttcanfd->data[idx + 2] << 16;
		case 2:
			msg_data += ttcanfd->data[idx + 1] << 8;
		case 1:
			msg_data += ttcanfd->data[idx + 0] << 0;
		}

		dbgprintf("T2: addr %p msg %x\n", (addr_in_msg_ram +
				(((idx >> 2) + 2) * CAN_WORD_IN_BYTES)),
				msg_data);
		writel(msg_data, (addr_in_msg_ram +
				(((idx >> 2) + 2) * CAN_WORD_IN_BYTES)));
		idx += 4;
		bytes_to_write -= 4;
	}

	return idx;
}

uint32_t ttcan_get_std_id_filter(struct ttcan_controller *ttcan, int idx)
{
	uint32_t filter_addr = ttcan->mram_base + ttcan->mram_sa.sidfc_flssa;

	uint32_t filter_offset = idx * SIDF_ELEM_SIZE;
	return readl(filter_addr + filter_offset);
}

uint64_t ttcan_get_xtd_id_filter(struct ttcan_controller *ttcan, int idx)
{
	uint64_t xtd;
	uint32_t filter_addr = ttcan->mram_base + ttcan->mram_sa.xidfc_flesa;
	uint32_t offset = idx * XIDF_ELEM_SIZE;

	xtd = ((uint64_t) readl(filter_addr + offset + CAN_WORD_IN_BYTES)) << 32;
	xtd |= readl(filter_addr + offset);
	return xtd;
}

void ttcan_tx_ded_msg_write(struct ttcan_controller *ttcan,
		struct canfd_frame *ttcanfd,
		uint8_t index, bool tt_en)
{
	uint32_t ram_addr = ttcan->mram_sa.txbc_tbsa +
		(index * ttcan->e_size.tx_buffer * CAN_WORD_IN_BYTES);

	ttcan_write_tx_msg_ram(ttcan, ram_addr, ttcanfd, index);
}

void ttcan_tx_trigger_msg_transmit(struct ttcan_controller *ttcan, uint8_t index)
{
	ttcan_write32(ttcan, ADR_MTTCAN_TXBAR, (1 << index));
}

int ttcan_tx_msg_buffer_write(struct ttcan_controller *ttcan,
		struct canfd_frame *ttcanfd, bool tt_en)
{
	int msg_id = 0;
	uint32_t txbrp_reg = ttcan_read32(ttcan, ADR_MTTCAN_TXBRP);
	uint32_t txbrp_free = ~txbrp_reg;

	/* number of buffers to consider */
	txbrp_free &= (1 << ttcan->tx_config.ded_buff_num) - 1;

	/* find first free buffer */
	msg_id = ffs(txbrp_free) - 1;

	if (msg_id < 0)
		return -ENOMEM;

	ttcan_tx_ded_msg_write(ttcan, ttcanfd, msg_id, tt_en);

	return msg_id;
}

int ttcan_set_tx_buffer_addr(struct ttcan_controller *ttcan, int num_buffer,
		int num_queue, enum ttcan_data_field_size dfs,
		int mode)
{
	int ret = 0;
	uint32_t txbc_reg;
	uint32_t txesc_reg;
	uint32_t rel_start_addr = ttcan->mram_sa.txbc_tbsa >> 2;

	if ((num_buffer + num_queue) > MAX_TX_BUFFER_ELEMS) {
		error_hook("Tx elements > MAX_TX_BUF");
		return -EINVAL;
	}
	ttcan->tx_config.ded_buff_num = num_buffer;
	ttcan->tx_config.fifo_q_num = num_queue;
	ttcan->tx_config.flags = mode;
	ttcan->tx_config.dfs = dfs;

	txbc_reg = (rel_start_addr << MTT_TXBC_TBSA_SHIFT) & MTT_TXBC_TBSA_MASK;
	txbc_reg |= (num_buffer << MTT_TXBC_NDTB_SHIFT) & MTT_TXBC_NDTB_MASK;
	txbc_reg |= (num_queue << MTT_TXBC_TFQS_SHIFT) & MTT_TXBC_TFQS_MASK;

	if (mode & 0x1)
		txbc_reg |= MTT_TXBC_TFQM_MASK;	/* Queue mode */
	else
		txbc_reg &= ~(MTT_TXBC_TFQM_MASK);	/* FIFO mode */

	ret = ttcan_write32_check(ttcan, ADR_MTTCAN_TXBC, txbc_reg,
			MTTCAN_TXBC_MSK);
	if (ret) {
		error_hook("Error in setting ADR_MTTCAN_TXBC");
		return ret;
	}

	txesc_reg = (dfs << MTT_TXESC_TBDS_SHIFT) & MTT_TXESC_TBDS_MASK;

	ret = ttcan_write32_check(ttcan, ADR_MTTCAN_TXESC, txesc_reg,
			MTTCAN_TXESC_MSK);
	if (ret) {
		error_hook("Error in setting ADR_MTTCAN_TXESC");
		return ret;
	}

	/* Enable TC interrupt for tx buffers + queue */
	ttcan_write32(ttcan, ADR_MTTCAN_TXBTIE,
			((1 << (num_buffer + num_queue)) - 1));
	/* Enable TCF interrupt for tx buffers */
	ttcan_write32(ttcan, ADR_MTTCAN_TXBCIE,
			((1 << (num_buffer + num_queue)) - 1));

	/* Populate buffersize in data structure e_size in words */
	ttcan->e_size.tx_buffer =
		TX_BUF_ELEM_HEADER_WORD + (data_in_element(dfs) >> 2);
	return ret;
}

/* Queue Message in Tx Queue
 * Return
 *	-ve in case of error
 *      idx written buffer index
 */
int ttcan_tx_fifo_queue_msg(struct ttcan_controller *ttcan,
		struct canfd_frame *ttcanfd)
{
	uint32_t txfqs_reg;
	uint32_t put_idx;

	txfqs_reg = ttcan_read32(ttcan, ADR_MTTCAN_TXFQS);
	put_idx = (txfqs_reg & MTT_TXFQS_TFQPI_MASK) >> MTT_TXFQS_TFQPI_SHIFT;
	if ((txfqs_reg & MTT_TXFQS_TFQF_MASK) == 0) {
		ttcan_tx_ded_msg_write(ttcan, ttcanfd, put_idx, 0);
		return put_idx;
	}

	error_hook("Tx queue/FIFO full");
	return -ENOMEM;
}

/* Check tx fifo status
 *  return 1 if fifo full
 */
int ttcan_tx_fifo_full(struct ttcan_controller *ttcan)
{
	uint32_t txfqs_reg;

	txfqs_reg = ttcan_read32(ttcan, ADR_MTTCAN_TXFQS);
	return (txfqs_reg & MTT_TXFQS_TFQF_MASK) >> MTT_TXFQS_TFQF_SHIFT;
}

/* Rx Buff Section */
void ttcan_set_rx_buffer_addr(struct ttcan_controller *ttcan)
{
	uint32_t rxbc_reg = 0;
	uint32_t relative_addr;

	relative_addr = (ttcan->mram_sa.rxbc_rbsa - ttcan->mram_sa.base) >> 2;
	rxbc_reg = (relative_addr << MTT_RXBC_RBSA_SHIFT) & MTT_RXBC_RBSA_MASK;

	ttcan_write32(ttcan, ADR_MTTCAN_RXBC, rxbc_reg);
}

static int process_rx_mesg(struct ttcan_controller *ttcan, uint32_t addr)
{
	int ret;
	struct mttcanfd_frame fd;

	fd.cmdid = MTTCAN_MSG_RX;
	ttcan_raw_read_rx_msg_ram(ttcan, addr, &fd.payload.data[0]);
	ret = tegra_add_msg_ctlr_list(ttcan, &fd);
	if (ret < 0) {
		ttcan->stats.rx_dropped++;
		return -ENOMEM;
	}

	ttcan->stats.rx_packets++;
	ttcan->stats.rx_bytes += fd.payload.frame.d_len;
	return 0;
}

int ttcan_read_rx_buffer(struct ttcan_controller *ttcan)
{
	uint32_t ndat1, ndat2;
	uint32_t read_addr;
	int msgs_read = 0;

	ndat1 = ttcan_read32(ttcan, ADR_MTTCAN_NDAT1);
	ndat2 = ttcan_read32(ttcan, ADR_MTTCAN_NDAT2);

	while (ndat1 != 0 || ndat2 != 0) {
		uint32_t bit_set1 = ffs(ndat1) - 1;
		uint32_t bit_set2 = ffs(ndat2) - 1;
		if (ndat1) {
			read_addr = ttcan->mram_sa.rxbc_rbsa + (bit_set1 *
				ttcan->e_size.rx_buffer * CAN_WORD_IN_BYTES);
			read_addr += ttcan->mram_sa.virt_base - ttcan->mram_sa.base;
			if (process_rx_mesg(ttcan, read_addr))
				return msgs_read;
			ttcan_write32(ttcan, ADR_MTTCAN_NDAT1, (BIT(bit_set1)));
			msgs_read++;
		}

		if (ndat2) {
			read_addr = ttcan->mram_sa.rxbc_rbsa + (bit_set2 *
				ttcan->e_size.rx_buffer * CAN_WORD_IN_BYTES);
			read_addr += ttcan->mram_sa.virt_base - ttcan->mram_sa.base;
			if (process_rx_mesg(ttcan, read_addr))
				return msgs_read;
			ttcan_write32(ttcan, ADR_MTTCAN_NDAT2, BIT(bit_set2));
			msgs_read++;
		}
		ndat1 &= ~(BIT(bit_set1));
		ndat2 &= ~(BIT(bit_set2));
	}

	return msgs_read;
}

/* Tx Evt Fifo */

unsigned int ttcan_read_txevt_fifo(struct ttcan_controller *ttcan,
		struct mttcanfd_frame *fd)
{
	struct mttcan_tx_evt_element txevt;
	uint32_t txefs;
	uint32_t read_addr;
	int q_read = 0;
	int msgs_read = 0;

	txefs = ttcan_read32(ttcan, ADR_MTTCAN_TXEFS);

	if (!(txefs & MTT_TXEFS_EFFL_MASK)) {
		printf("%s: Tx Event FIFO empty\n", __func__);
		return 0;
	}
	q_read = ttcan->tx_config.evt_q_num;
	while ((txefs & MTT_TXEFS_EFFL_MASK) && q_read--) {

		uint32_t get_idx =
		    (txefs & MTT_TXEFS_EFGI_MASK) >> MTT_TXEFS_EFGI_SHIFT;
		read_addr =
		    ttcan->mram_sa.txefc_efsa +
		    (get_idx * TX_EVENT_FIFO_ELEM_SIZE);

		dbgprintf("%s:txevt: read_addr %x EFGI %x\n", __func__,
			 read_addr, get_idx);

		ttcan_read_txevt_ram(ttcan, read_addr, &txevt);

		fd->cmdid = MTTCAN_MSG_TXEVT;
		fd->payload.data[0] = txevt.f0;
		fd->payload.data[1] = txevt.f1;
		if (tegra_add_msg_ctlr_list(ttcan, fd) < 0)
			error_hook("Failed to send message");

		ttcan_write32(ttcan, ADR_MTTCAN_TXEFA, get_idx);
		txefs = ttcan_read32(ttcan, ADR_MTTCAN_TXEFS);
		msgs_read++;
	}
	return msgs_read;
}

/* Rx FIFO section */

unsigned int ttcan_read_rx_fifo0(struct ttcan_controller *ttcan)
{
	uint32_t rxf0s_reg;
	struct mttcanfd_frame fd;
	uint32_t read_addr;
	int q_read = 0;
	unsigned int msgs_read = 0;

	rxf0s_reg = ttcan_read32(ttcan, ADR_MTTCAN_RXF0S);

	if (!(rxf0s_reg & MTT_RXF0S_F0FL_MASK)) {
		return msgs_read;
	}

	/* Read at max queue size in one attempt */
	q_read = ttcan->rx_config.rxq0_size;

	while ((rxf0s_reg & MTT_RXF0S_F0FL_MASK) && q_read--) {
		uint32_t get_idx = (rxf0s_reg & MTT_RXF0S_F0GI_MASK) >>
			MTT_RXF0S_F0GI_SHIFT;
		if (ttcan->rx_config.rxq0_bmsk & BIT(get_idx)) {
			/* All ready process on High priority */
			ttcan_write32(ttcan, ADR_MTTCAN_RXF0A, get_idx);
			ttcan->rx_config.rxq0_bmsk &= ~(1U << get_idx);
			rxf0s_reg = ttcan_read32(ttcan, ADR_MTTCAN_RXF0S);
			continue;
		}

		read_addr = ttcan->mram_sa.rxf0c_f0sa +
			(get_idx * ttcan->e_size.rx_fifo0 * CAN_WORD_IN_BYTES);
		read_addr += ttcan->mram_sa.virt_base - ttcan->mram_sa.base;

		dbgprintf("%s:fifo0: read_addr %x FOGI %x\n", __func__,
		   read_addr, get_idx);

		/* RX message ID */
		fd.cmdid = MTTCAN_MSG_RX;
		ttcan_raw_read_rx_msg_ram(ttcan, read_addr, &fd.payload.data[0]);

		/* Queue raw message to upper driver layers */
		if (mttcan_do_receive(ttcan, &fd) < 0)
			error_hook("Failed to send message");

		ttcan_write32(ttcan, ADR_MTTCAN_RXF0A, get_idx);
		rxf0s_reg = ttcan_read32(ttcan, ADR_MTTCAN_RXF0S);
		msgs_read++;
	}
	return msgs_read;
}

unsigned int ttcan_read_rx_fifo1(struct ttcan_controller *ttcan)
{
	uint32_t rxf1s_reg;
	struct mttcanfd_frame fd;
	uint32_t read_addr;
	int q_read = 0;
	int msgs_read = 0;

	rxf1s_reg = ttcan_read32(ttcan, ADR_MTTCAN_RXF1S);

	if (!(rxf1s_reg & MTT_RXF1S_F1FL_MASK)) {
		return msgs_read;
	}

	/* Read at max queue size in one attempt */
	q_read = ttcan->rx_config.rxq1_size;

	while ((rxf1s_reg & MTT_RXF1S_F1FL_MASK) && q_read--) {
		uint32_t get_idx = (rxf1s_reg & MTT_RXF1S_F1GI_MASK) >>
			MTT_RXF1S_F1GI_SHIFT;
		if (ttcan->rx_config.rxq1_bmsk & BIT(get_idx)) {
			/* All ready process on High priority */
			ttcan_write32(ttcan, ADR_MTTCAN_RXF1A, get_idx);
			ttcan->rx_config.rxq1_bmsk &= ~(1U << get_idx);
			rxf1s_reg = ttcan_read32(ttcan, ADR_MTTCAN_RXF1S);
			continue;
		}

		read_addr = ttcan->mram_sa.rxf1c_f1sa +
			(get_idx * ttcan->e_size.rx_fifo1 * CAN_WORD_IN_BYTES);
		read_addr += ttcan->mram_sa.virt_base - ttcan->mram_sa.base;

		dbgprintf("%s:fifo1: read_addr %x FOGI %x\n", __func__,
				read_addr, get_idx);

		/* RX message ID */
		fd.cmdid = MTTCAN_MSG_RX;
		ttcan_raw_read_rx_msg_ram(ttcan, read_addr, &fd.payload.data[0]);

		/* Queue raw message to upper driver layers */
		if (mttcan_do_receive(ttcan, &fd) < 0)
			error_hook("Failed to send message");

		ttcan_write32(ttcan, ADR_MTTCAN_RXF1A, get_idx);
		rxf1s_reg = ttcan_read32(ttcan, ADR_MTTCAN_RXF1S);
		msgs_read++;
	}
	return msgs_read;
}

/* Returns message read else return 0 */
unsigned int ttcan_read_rx_fifo(struct ttcan_controller *ttcan)
{
	int msgs_read = 0;

	msgs_read = ttcan_read_rx_fifo0(ttcan);

	if (ttcan->rx_config.rxq1_size)
		msgs_read += ttcan_read_rx_fifo1(ttcan);

	return msgs_read;
}

unsigned int ttcan_read_hp_mesgs(struct ttcan_controller *ttcan,
		struct mttcanfd_frame *fd)
{
	uint32_t hpms;
	uint32_t fltr_idx;
	uint32_t buf_idx;
	uint32_t msi;
	uint32_t read_addr;

	hpms = ttcan_read32(ttcan, ADR_MTTCAN_HPMS);
	fltr_idx = (hpms & MTT_HPMS_FIDX_MASK) >> MTT_HPMS_FIDX_SHIFT;
	buf_idx = (hpms & MTT_HPMS_BIDX_MASK) >> MTT_HPMS_BIDX_SHIFT;
	msi = (hpms & MTT_HPMS_MSI_MASK) >> MTT_HPMS_MSI_SHIFT;

	if (hpms & MTT_HPMS_FLST_MASK) {
		/* Extended filter list */
		dbgprintf("Xtd Filter:%d Matched\r\n", fltr_idx);
		dbgprintf("0x%llx\r\n", ttcan_get_xtd_id_filter(ttcan, fltr_idx));
	} else {
		/* Standard filter list */
		dbgprintf("Std Filter:%d Matched\r\n", fltr_idx);
		dbgprintf("0x%x\r\n", ttcan_get_std_id_filter(ttcan, fltr_idx));
	}

	switch (msi) {
	default:
		dbgprintf("High Priority Interrupt received, no Mesg\r\n");
		return 0;
	case 1:
		dbgprintf("High Priority FIFO Mesg lost\r\n");
		return 0;
	case 2:
		read_addr = ttcan->mram_sa.rxf0c_f0sa +
			buf_idx * (ttcan->e_size.rx_fifo0 << 2);
		read_addr += ttcan->mram_sa.virt_base - ttcan->mram_sa.base;
		fd->cmdid = MTTCAN_MSG_RX;
		ttcan_raw_read_rx_msg_ram(ttcan, read_addr, &fd->payload.data[0]);
		ttcan->rx_config.rxq0_bmsk |= 1 << buf_idx;
		break;
	case 3:
		read_addr = ttcan->mram_sa.rxf1c_f1sa +
			buf_idx * (ttcan->e_size.rx_fifo1 << 2);
		read_addr += ttcan->mram_sa.virt_base - ttcan->mram_sa.base;
		fd->cmdid = MTTCAN_MSG_RX;
		ttcan_raw_read_rx_msg_ram(ttcan, read_addr, &fd->payload.data[0]);
		ttcan->rx_config.rxq1_bmsk |= 1 << buf_idx;
		break;
	}
	return 1;
}

void ttcan_set_rx_fifo0(struct ttcan_controller *ttcan, int num_elems,
		int fifo_watermark)
{
	uint32_t rel_phy_addr;
	uint32_t rxf0c_reg = 0;

	rel_phy_addr = (ttcan->mram_sa.rxf0c_f0sa - ttcan->mram_sa.base) >> 2;
	rxf0c_reg = (fifo_watermark << MTT_RXF0C_F0WM_SHIFT) &
		MTT_RXF0C_F0WM_MASK;
	rxf0c_reg |= (num_elems << MTT_RXF0C_F0S_SHIFT) & MTT_RXF0C_F0S_MASK;
	rxf0c_reg |= (rel_phy_addr << MTT_RXF0C_F0SA_SHIFT) &
		MTT_RXF0C_F0SA_MASK;

	ttcan_write32(ttcan, ADR_MTTCAN_RXF0C, rxf0c_reg);

	ttcan->rx_config.rxq0_size = num_elems;
}

void ttcan_set_rx_fifo1(struct ttcan_controller *ttcan, int num_elems,
		int fifo_watermark)
{
	uint32_t rel_phy_addr;
	uint32_t rxf1c_reg = 0;

	rel_phy_addr = (ttcan->mram_sa.rxf1c_f1sa - ttcan->mram_sa.base) >> 2;
	rxf1c_reg = (fifo_watermark << MTT_RXF1C_F1WM_SHIFT) &
		MTT_RXF1C_F1WM_MASK;
	rxf1c_reg |= (num_elems << MTT_RXF1C_F1S_SHIFT) & MTT_RXF1C_F1S_MASK;
	rxf1c_reg |= (rel_phy_addr << MTT_RXF1C_F1SA_SHIFT) &
		MTT_RXF1C_F1SA_MASK;

	ttcan_write32(ttcan, ADR_MTTCAN_RXF1C, rxf1c_reg);

	ttcan->rx_config.rxq1_size = num_elems;
}

void ttcan_config_rx_data_elem_sizes(struct ttcan_controller *ttcan,
		enum ttcan_data_field_size rxbuf_dfs,
		enum ttcan_data_field_size rxfifo0_dfs,
		enum ttcan_data_field_size rxfifo1_dfs)
{
	uint32_t rxesc_reg = 0;

	rxesc_reg = (rxbuf_dfs << MTT_RXESC_RBDS_SHIFT) & MTT_RXESC_RBDS_MASK;
	rxesc_reg |= (rxfifo0_dfs << MTT_RXESC_F0DS_SHIFT) &
		MTT_RXESC_F0DS_MASK;
	rxesc_reg |= (rxfifo1_dfs << MTT_RXESC_F1DS_SHIFT) &
		MTT_RXESC_F1DS_MASK;

	ttcan_write32(ttcan, ADR_MTTCAN_RXESC, rxesc_reg);

	ttcan->e_size.rx_buffer =
		RX_BUF_ELEM_HEADER_WORD + (data_in_element(rxbuf_dfs) >> 2);
	ttcan->e_size.rx_fifo0 =
		RX_BUF_ELEM_HEADER_WORD + (data_in_element(rxfifo0_dfs) >> 2);
	ttcan->e_size.rx_fifo1 =
		RX_BUF_ELEM_HEADER_WORD + (data_in_element(rxfifo1_dfs) >> 2);
}

/*  Filters Section */

int ttcan_set_gfc(struct ttcan_controller *ttcan, uint32_t regval)
{
	int ret = 0;

	ret = ttcan_write32_check(ttcan, ADR_MTTCAN_GFC, regval,
			MTTCAN_GFC_MSK);
	if (ret)
		error_hook("unable to set GFC register");

	return ret;
}

uint32_t ttcan_get_gfc(struct ttcan_controller *ttcan)
{
	return ttcan_read32(ttcan, ADR_MTTCAN_GFC);
}

int ttcan_set_xidam(struct ttcan_controller *ttcan, uint32_t regval)
{
	int ret = 0;

	ret = ttcan_write32_check(ttcan, ADR_MTTCAN_XIDAM, regval,
			MTTCAN_XIDAM_MSK);
	if (ret)
		error_hook("unable to set XIDAM register");
	return ret;
}

uint32_t ttcan_get_xidam(struct ttcan_controller *ttcan)
{
	return ttcan_read32(ttcan, ADR_MTTCAN_XIDAM);
}

void ttcan_set_std_id_filter_addr(struct ttcan_controller *ttcan,
		uint32_t list_size)
{
	uint32_t sidfc_reg = 0;
	uint32_t rel_start_addr = ttcan->mram_sa.sidfc_flssa >> 2;

	if (list_size > 128)
		list_size = 128;

	sidfc_reg = (rel_start_addr << MTT_SIDFC_FLSSA_SHIFT) &
		MTT_SIDFC_FLSSA_MASK;
	sidfc_reg |= (list_size << MTT_SIDFC_LSS_SHIFT) & MTT_SIDFC_LSS_MASK;
	ttcan_write32(ttcan, ADR_MTTCAN_SIDFC, sidfc_reg);
}

void ttcan_set_xtd_id_filter_addr(struct ttcan_controller *ttcan,
		uint32_t list_size)
{
	uint32_t xidfc_reg = 0;
	uint32_t rel_start_addr = ttcan->mram_sa.xidfc_flesa >> 2;

	if (list_size > 64)
		list_size = 64;

	xidfc_reg = (rel_start_addr << MTT_XIDFC_FLESA_SHIFT) &
		MTT_XIDFC_FLESA_MASK;
	xidfc_reg |= (list_size << MTT_XIDFC_LSE_SHIFT) & MTT_XIDFC_LSE_MASK;
	ttcan_write32(ttcan, ADR_MTTCAN_XIDFC, xidfc_reg);

}

void ttcan_set_time_stamp_conf(struct ttcan_controller *ttcan,
		uint16_t timer_prescalar, enum ttcan_timestamp_source timer_type)
{
	uint32_t tscc = 0;

	if (timer_prescalar > 15)
		timer_prescalar = 15;

	tscc = (timer_prescalar << MTT_TSCC_TCP_SHIFT) & MTT_TSCC_TCP_MASK;
	tscc |= (timer_type << MTT_TSCC_TSS_SHIFT) & MTT_TSCC_TSS_MASK;
	ttcan_write32(ttcan, ADR_MTTCAN_TSCC, tscc);
	ttcan->ts_prescalar = timer_prescalar + 1;
}

void ttcan_reset_std_id_filter(struct ttcan_controller *ttcan,
		uint32_t list_size)
{
	int idx;
	uint32_t filter_addr = ttcan->mram_sa.sidfc_flssa + ttcan->mram_sa.base;

	for (idx = 0; idx < list_size; idx++) {
		uint32_t offset = idx * SIDF_ELEM_SIZE;

		writel(0, (filter_addr + offset));
	}
}

void ttcan_reset_xtd_id_filter(struct ttcan_controller *ttcan,
		uint32_t list_size)
{
	int idx;
	uint32_t filter_addr = ttcan->mram_sa.xidfc_flesa + ttcan->mram_sa.base;

	for (idx = 0; idx < list_size; idx++) {
		uint32_t offset = idx * XIDF_ELEM_SIZE;

		writel(0, (filter_addr + offset));
		writel(0, (filter_addr + offset +
			CAN_WORD_IN_BYTES));
	}
}

void ttcan_set_txevt_fifo_conf(struct ttcan_controller *ttcan,
		uint8_t water_mark, uint8_t size)
{
	uint32_t txefc = 0;
	uint32_t rel_addr = ttcan->mram_sa.txefc_efsa >> 2;

	if (water_mark < 32)
		txefc = (water_mark << MTT_TXEFC_EFWM_SHIFT) &
			MTT_TXEFC_EFWM_MASK;

	txefc |= size << MTT_TXEFC_EFS_SHIFT & MTT_TXEFC_EFS_MASK;
	txefc |= rel_addr << MTT_TXEFC_EFSA_SHIFT & MTT_TXEFC_EFSA_MASK;
	ttcan_write32(ttcan, ADR_MTTCAN_TXEFC, txefc);
	ttcan->tx_config.evt_q_num = size;
}

void ttcan_set_xtd_mask_add(struct ttcan_controller *ttcan, int extid_mask)
{
	uint32_t xidam_reg = 0;

	xidam_reg = (extid_mask & MTT_XIDAM_EIDM_MASK) << MTT_XIDAM_EIDM_SHIFT;
	ttcan_write32(ttcan, ADR_MTTCAN_XIDAM, xidam_reg);
}

uint32_t ttcan_read_tx_complete_reg(struct ttcan_controller *ttcan)
{
	return ttcan_read32(ttcan, ADR_MTTCAN_TXBTO);
}

void ttcan_set_tx_cancel_request(struct ttcan_controller *ttcan, uint32_t txbcr)
{
	ttcan_write32(ttcan, ADR_MTTCAN_TXBCR, txbcr);
}

uint32_t ttcan_read_tx_cancelled_reg(struct ttcan_controller *ttcan)
{
	return ttcan_read32(ttcan, ADR_MTTCAN_TXBCF);
}

uint32_t ttcan_read_psr(struct ttcan_controller *ttcan)
{
	return ttcan_read32(ttcan, ADR_MTTCAN_PSR);
}

void ttcan_clear_intr(struct ttcan_controller *ttcan)
{
	ttcan_write32(ttcan, ADR_MTTCAN_IR, 0xFFFFFFFF);
}

void ttcan_clear_tt_intr(struct ttcan_controller *ttcan)
{
	ttcan_write32(ttcan, ADR_MTTCAN_TTIR, 0xFFFFFFFF);
}

uint32_t ttcan_read_ir(struct ttcan_controller *ttcan)
{
	return ttcan_read32(ttcan, ADR_MTTCAN_IR);
}

void ttcan_ir_write(struct ttcan_controller *ttcan, uint32_t value)
{
	ttcan_write32(ttcan, ADR_MTTCAN_IR, value);
}

void ttcan_ttir_write(struct ttcan_controller *ttcan, uint32_t value)
{
	ttcan_write32(ttcan, ADR_MTTCAN_TTIR, value);
}

uint32_t ttcan_read_ttir(struct ttcan_controller *ttcan)
{
	return ttcan_read32(ttcan, ADR_MTTCAN_TTIR);
}

void ttcan_ttier_write(struct ttcan_controller *ttcan, uint32_t val)
{
	ttcan_write32(ttcan, ADR_MTTCAN_TTIE, val);
}

void ttcan_select_enable_intr(struct ttcan_controller *ttcan, uint32_t value)
{
	ttcan_write32(ttcan, ADR_MTTCAN_IE, value);
	ttcan_write32(ttcan, ADR_MTTCAN_ILE, CAN_ILE_0 | CAN_ILE_1);
}

int ttcan_mesg_ram_config(struct ttcan_controller *ttcan)
{
	ttcan->mram_base = ttcan->mram_sa.base;

	if ((REL_TMC_END >= RAM_BYTES_PER_MTT_CAN)) {
		error_hook("Incorrect config for Message RAM");
		return -EINVAL;
	}

	ttcan->mram_sa.sidfc_flssa = REL_SIDFC_FLSSA;
	ttcan->mram_sa.xidfc_flesa = REL_XIDFC_FLESA;
	ttcan->mram_sa.rxf0c_f0sa = REL_RXF0C_F0SA;
	ttcan->mram_sa.rxf1c_f1sa = REL_RXF1C_F1SA;
	ttcan->mram_sa.rxbc_rbsa = REL_RXBC_RBSA;
	ttcan->mram_sa.txefc_efsa = REL_TXEFC_EFSA;
	ttcan->mram_sa.txbc_tbsa = REL_TXBC_TBSA;
	ttcan->mram_sa.tmc_tmsa = REL_TMC_TMSA;

	dbgprintf("\tMessage RAM Configuration CAN%d\r\n"
		"\t| base addr   |0x%08x|\r\n"
		"\t| sidfc_flssa |0x%08x|\r\n\t| xidfc_flesa |0x%08x|\r\n"
		"\t| rxf0c_f0sa  |0x%08x|\r\n\t| rxf1c_f1sa  |0x%08x|\r\n"
		"\t| rxbc_rbsa   |0x%08x|\r\n\t| txefc_efsa  |0x%08x|\r\n"
		"\t| txbc_tbsa   |0x%08x|\r\n\t| tmc_tmsa    |0x%08x|\r\n",
		ttcan->id, ttcan->mram_sa.base, ttcan->mram_sa.sidfc_flssa,
		ttcan->mram_sa.xidfc_flesa, ttcan->mram_sa.rxf0c_f0sa,
		ttcan->mram_sa.rxf1c_f1sa, ttcan->mram_sa.rxbc_rbsa,
		ttcan->mram_sa.txefc_efsa, ttcan->mram_sa.txbc_tbsa,
		ttcan->mram_sa.tmc_tmsa);
	return 0;
}
