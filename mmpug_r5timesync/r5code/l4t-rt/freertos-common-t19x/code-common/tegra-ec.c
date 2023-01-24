/*
 * Copyright (c) 2018, NVIDIA CORPORATION.  All rights reserved.
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
#include <FreeRTOS.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <nvrm_drf.h>
#include <reg-access.h>
#include <err-hook.h>
#include <arsce_ec.h>
#include <tegra-ec.h>
#include <tegra-ec-priv.h>

#ifdef ENABLE_DEBUG
#define dbgprintf       printf
#define dbgprintf_isr   printf_isr
#else
static inline void dbgprintf(const char *fmt, ...)
{
}
static inline void dbgprintf_isr(const char *fmt, ...)
{
}
#endif

int tegra_ec_register_handler(
	const struct tegra_ec_id *id,
	uint32_t index,
	const struct tegra_ec_handler *handler)
{
	struct tegra_ec *dev = (struct tegra_ec *)id;

	/* Actual number of errors are total_errors +1 including EC internal error */
	if (index >= (dev->total_errors + 1)) {
		error_hookf("err index 0x%u exceeds max errors 0x%u", (unsigned)index,
						(unsigned)(dev->total_errors + 1));
		return -1;
	}

	portENTER_CRITICAL();

	dev->ec_handlers[index].mission_handler = handler->mission_handler;
	dev->ec_handlers[index].latent_handler = handler->latent_handler;
	dev->ec_handlers[index].data = handler->data;

	portEXIT_CRITICAL();

	return 0;
}

static void tegra_ec_initialize_handlers(const struct tegra_ec_id *id)
{
	uint32_t error_count;

	struct tegra_ec *dev = (struct tegra_ec *)id;

	for(error_count = 0; error_count < (dev->total_errors + 1); error_count++) {
		dev->ec_handlers[error_count].data = NULL;
		dev->ec_handlers[error_count].mission_handler = NULL;
		dev->ec_handlers[error_count].latent_handler = NULL;
	}
}

static void tegra_ec_slice_error_handler(const struct tegra_ec_id *id,
						uint32_t slice)
{
	uint32_t mask = 1;
	uint32_t err_count;

	struct tegra_ec *dev = (struct tegra_ec *)id;

	err_count = (slice * EC_ERRORS_PER_SLICE);

	/* Actual number of errors are total_errors +1 including EC internal error */
	if (err_count >= (dev->total_errors + 1))
		bug_hookf("err index %u exceeds max errors %u", (unsigned)err_count,
						(unsigned)(dev->total_errors + 1));

	while (mask != 0) {
		if (dev->latent_status & mask) {
			if (dev->ec_handlers[err_count].latent_handler != NULL) {
				dev->ec_handlers[err_count].
				latent_handler(dev->ec_handlers[err_count].data);
			}
		}

		if (dev->mission_status & mask) {
			if (dev->ec_handlers[err_count].mission_handler != NULL) {
				dev->ec_handlers[err_count].
				mission_handler(dev->ec_handlers[err_count].data);
			}
		}
		mask <<= 1;
		err_count++;
	}
}

void tegra_ec_error_handler(void *data)
{
	uint32_t slice = 0;
	uint32_t offset;
	const struct tegra_ec_id *id = (struct tegra_ec_id *)data;

	struct tegra_ec *dev = (struct tegra_ec *)id;

	while (slice < dev->total_slices) {

		offset = EC_LATENTERR_STATUS_OFFSET(slice);
		dev->latent_status = readl(id->base + offset);

		offset = EC_MISSIONERR_STATUS_OFFSET(slice);
		dev->mission_status = readl(id->base + offset);

		if ((dev->latent_status != 0U) || (dev->mission_status != 0U)) {
			dbgprintf_isr("slice 0x%u, mission_status 0x%u", (unsigned)slice,
						(unsigned)dev->mission_status);
			dbgprintf_isr("slice 0x%u, latent_status 0x%u", (unsigned)slice,
						(unsigned)dev->latent_status);
			tegra_ec_slice_error_handler(id, slice);
		}
		slice++;
	}

}

static int tegra_ec_validate_index(
	const struct tegra_ec_id *id,
	uint32_t *index,
	uint32_t *slice)
{

	struct tegra_ec *dev = (struct tegra_ec *)id;

	/* Actual number of errors are total_errors +1 including EC internal error */
	if (*index >= (dev->total_errors + 1)) {
		error_hookf("index 0x%u exceeds max limit 0x%u", (unsigned)index,
						(unsigned)(dev->total_errors + 1));
		return -1;
	}

	(*slice) = (*index) / 32;
	(*index) = (*index) % 32;

	return 0;
}

int tegra_ec_counter_reload(const struct tegra_ec_id *id, uint32_t index)
{
	uint32_t slice;
	uint32_t val = 0;
	uint32_t offset;

	if (tegra_ec_validate_index(id, &index, &slice) != 0U) {
		return -1;
	}

	offset = EC_COUNTER_RELOAD_OFFSET(slice);
	val |= (1 << index);
	writel(val, id->base + offset);

	return 0;
}

int tegra_ec_force_latent_error(const struct tegra_ec_id *id, uint32_t index)
{
	uint32_t slice;
	uint32_t val = 0;
	uint32_t offset;

	if (tegra_ec_validate_index(id, &index, &slice) != 0U) {
		return -1;
	}

	offset = EC_LATENTERR_FORCE_OFFSET(slice);
	val |= (1 << index);
	writel(val, id->base + offset);

	return 0;
}

int tegra_ec_clear_latent_error(const struct tegra_ec_id *id, uint32_t index)
{
	uint32_t slice;
	uint32_t val;
	uint32_t offset;

	if (tegra_ec_validate_index(id, &index, &slice) != 0U) {
		return -1;
	}

	/* SW must write 1 to clear the fields of this register.  */
	offset = EC_LATENTERR_STATUS_OFFSET(slice);
 	val = readl(id->base + offset);
	val |= (1 << index);
	writel(val, id->base + offset);

	return 0;
}

int tegra_ec_get_latent_error_status(
		const struct tegra_ec_id *id,
		uint32_t index,
		uint32_t *status)
{
	uint32_t slice;
	uint32_t val;
	uint32_t offset;

	if (tegra_ec_validate_index(id, &index, &slice) != 0U) {
		return -1;
	}

	/* Get the status of latent error at 'index' */
	offset = EC_LATENTERR_STATUS_OFFSET(slice);
 	val = readl(id->base + offset);
	if (val & (1 << index))
		(*status) = 1;
	else
		(*status) = 0;

	return 0;
}

int tegra_ec_disable_latent_error(const struct tegra_ec_id *id, uint32_t index)
{
	uint32_t slice;
	uint32_t val;
	uint32_t offset;

	if (tegra_ec_validate_index(id, &index, &slice) != 0U) {
		return -1;
	}

	offset = EC_LATENTERR_ENABLE_OFFSET(slice);
 	val = readl(id->base + offset);
	val &= ~(1 << index);
	writel(val, id->base + offset);

	return 0;
}

int tegra_ec_enable_latent_error(const struct tegra_ec_id *id, uint32_t index)
{
	uint32_t slice;
	uint32_t val;
	uint32_t offset;

	if (tegra_ec_validate_index(id, &index, &slice) != 0U) {
		return -1;
	}

	offset = EC_LATENTERR_ENABLE_OFFSET(slice);
 	val = readl(id->base + offset);
	val |= (1 << index);
	writel(val, id->base + offset);

	return 0;
}

int tegra_ec_clear_inject_mission_error(const struct tegra_ec_id *id, uint32_t index)
{
	uint32_t slice;
	uint32_t val;
	uint32_t offset;

	if (tegra_ec_validate_index(id, &index, &slice) != 0U) {
		return -1;
	}

	if ((id->error_inject_mask[slice] & (1 << index)) == 0U) {
		error_hookf("failed, index 0x%u does not have error injection", (unsigned)index);
		return -1;
	}

	offset = EC_MISSIONERR_INJECT_OFFSET(slice);
 	val = readl(id->base + offset);
	val &= ~(1 << index);
	writel(val, id->base + offset);

	return 0;
}

int tegra_ec_set_inject_mission_error(const struct tegra_ec_id *id, uint32_t index)
{
	uint32_t slice;
	uint32_t val;
	uint32_t offset;

	if (tegra_ec_validate_index(id, &index, &slice) != 0U) {
		return -1;
	}

	if ((id->error_inject_mask[slice] & (1 << index)) == 0U) {
		error_hookf("failed, index 0x%u does not have error injection", (unsigned)index);
		return -1;
	}

	offset = EC_MISSIONERR_INJECT_OFFSET(slice);
 	val = readl(id->base + offset);
	val |= (1 << index);
	writel(val, id->base + offset);

	return 0;
}

int tegra_ec_force_mission_error(const struct tegra_ec_id *id, uint32_t index)
{
	uint32_t slice;
	uint32_t val = 0;
	uint32_t offset;

	if (tegra_ec_validate_index(id, &index, &slice) != 0UL) {
		return -1;
	}

	offset = EC_MISSIONERR_FORCE_OFFSET(slice);
	val |= (1 << index);
	writel(val, id->base + offset);

	return 0;
}

int tegra_ec_clear_mission_error(const struct tegra_ec_id *id, uint32_t index)
{
	uint32_t slice;
	uint32_t val;
	uint32_t offset;

	if (tegra_ec_validate_index(id, &index, &slice) != 0UL) {
		return -1;
	}

	offset = EC_MISSIONERR_STATUS_OFFSET(slice);

	/* SW must write 1 to clear the fields of this register.  */
 	val = readl(id->base + offset);
	val |= (1 << index);
	writel(val, id->base + offset);

	return 0;
}

int tegra_ec_get_mission_error_status(
		const struct tegra_ec_id *id,
		uint32_t index,
		uint32_t *status)
{
	uint32_t slice;
	uint32_t val;
	uint32_t offset;

	if (tegra_ec_validate_index(id, &index, &slice) != 0UL) {
		return -1;
	}

	/* Get the status of mission error at 'index' */
	offset = EC_MISSIONERR_STATUS_OFFSET(slice);
 	val = readl(id->base + offset);
	if (val & (1 << index))
		(*status) = 1;
	else
		(*status) = 0;

	return 0;
}

int tegra_ec_disable_mission_error(const struct tegra_ec_id *id, uint32_t index)
{
	uint32_t slice;
	uint32_t val;
	uint32_t offset;

	if (tegra_ec_validate_index(id, &index, &slice) != 0UL) {
		return -1;
	}

	offset = EC_MISSIONERR_ENABLE_OFFSET(slice);
 	val = readl(id->base + offset);
	val &= ~(1 << index);
	writel(val, id->base + offset);

	return 0;
}

int tegra_ec_enable_mission_error(const struct tegra_ec_id *id, uint32_t index)
{
	uint32_t slice;
	uint32_t val;
	uint32_t offset;

	if (tegra_ec_validate_index(id, &index, &slice) != 0UL) {
		return -1;
	}

	offset = EC_MISSIONERR_ENABLE_OFFSET(slice);
 	val = readl(id->base + offset);
	val |= (1 << index);
	writel(val, id->base + offset);

	return 0;
}

int tegra_ec_get_sec_threshold(const struct tegra_ec_id *id)
{
	struct tegra_ec *dev = (struct tegra_ec *)id;

	return dev->sec_threshold;
}

int tegra_ec_set_sec_threshold(const struct tegra_ec_id *id, uint32_t threshold)
{
	uint32_t val = 0;

	struct tegra_ec *dev = (struct tegra_ec *)id;

	if (threshold > MAX_EC_SEC_THRESHOLD) {
		error_hookf("threshold 0x%u exceeds max limit 0x%u", (unsigned)threshold, MAX_EC_SEC_THRESHOLD);
		return -1;
	}

	val = NV_FLD_SET_DRF_NUM(SCE_EC_REGS, CORRECTABLE_THRESHOLD, COUNT, threshold, val);
	writel(val, id->base + EC_CORRECTABLE_THRESHOLD_OFFSET);

	dev->sec_threshold = threshold;

	return 0;
}

int tegra_ec_read_error(
	const struct tegra_ec_id *id,
	uint32_t index,
	struct tegra_ec_error *error)
{
	uint32_t val;

	struct tegra_ec *dev = (struct tegra_ec *)id;

	/* Actual number of errors are total_errors +1 including EC internal error */
	if (index >= (dev->total_errors + 1)) {
		error_hookf("index 0x%u exceeds max limit 0x%u",(unsigned)index,
						(unsigned)(dev->total_errors + 1));
		return -1;
	}

	writel(index, id->base + EC_MISSIONERR_INDEX_OFFSET);

	error->type = readl(id->base + EC_MISSIONERR_TYPE_OFFSET);
	val = readl(id->base + EC_CURRENT_COUNTER_VALUE_OFFSET);
	error->counter = NV_DRF_VAL(SCE_EC_REGS, CURRENT_COUNTER_VALUE, VALUE, val);
	if (id->user_value_support == 1U)
		error->user_value = readl(id->base + EC_MISSIONERR_USERVALUE_OFFSET);

	return 0;
}

int tegra_ec_sw_reset(const struct tegra_ec_id *id)
{
	uint32_t val = 0;

	struct tegra_ec *dev = (struct tegra_ec *)id;

	val = NV_FLD_SET_DRF_NUM(SCE_EC_REGS, SWRESET, SWRST, 1, val);
	writel(val, id->base + EC_SWRESET_OFFSET);

	val = readl(id->base + EC_CORRECTABLE_THRESHOLD_OFFSET);
	dev->sec_threshold = NV_DRF_VAL(SCE_EC_REGS, CORRECTABLE_THRESHOLD, COUNT, val);
	dev->error_inject_lock = 1;

	dev->sw_reset_count++;

	return 0;
}

int tegra_ec_err_inject_lock(const struct tegra_ec_id *id)
{
	uint32_t val;

	/* struct tegra_ec_id assumed to be first field in device struct tegra_ec */
	struct tegra_ec *dev = (struct tegra_ec *)id;

	if (dev->error_inject_lock)
		goto end;   /* already locked */

	val = readl(id->base + EC_MISSIONERR_INJECT_UNLOCK_OFFSET);
	val = NV_FLD_SET_DRF_DEF(SCE_EC_REGS, MISSIONERR_INJECT_UNLOCK, VALUE, LOCK, val);
	writel(val, id->base + EC_MISSIONERR_INJECT_UNLOCK_OFFSET);

	dev->error_inject_lock = 1;

end:
	return 0;
}

int tegra_ec_err_inject_unlock(const struct tegra_ec_id *id)
{
	uint32_t val;

	struct tegra_ec *dev = (struct tegra_ec *)id;

	if (!dev->error_inject_lock)
		goto end;   /* already unlocked */

	val = readl(id->base + EC_MISSIONERR_INJECT_UNLOCK_OFFSET);
	val = NV_FLD_SET_DRF_DEF(SCE_EC_REGS, MISSIONERR_INJECT_UNLOCK, VALUE, UNLOCK, val);
	writel(val, id->base + EC_MISSIONERR_INJECT_UNLOCK_OFFSET);

	dev->error_inject_lock = 0;

end:
	return 0;
}

int tegra_ec_init(const struct tegra_ec_id *id)
{
	uint32_t val;

	/* struct tegra_ec_id is the first field in device struct tegra_ec */
	struct tegra_ec *dev = (struct tegra_ec *)id;

	val = readl(id->base + EC_FEATURE_OFFSET);
	dev->total_errors = NV_DRF_VAL(SCE_EC_REGS, FEATURE, NUM_ERR, val);
	dev->total_slices = NV_DRF_VAL(SCE_EC_REGS, FEATURE, NUM_ERR_SLICES, val);

	val = readl(id->base + EC_CORRECTABLE_THRESHOLD_OFFSET);
	dev->sec_threshold = NV_DRF_VAL(SCE_EC_REGS, CORRECTABLE_THRESHOLD, COUNT, val);

	dev->sw_reset_count = 0;
	dev->error_inject_lock = 1;

	tegra_ec_initialize_handlers(id);

	return 0;
}
