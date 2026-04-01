/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nuvoton_numaker_pdma

#include <errno.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/clock_control_numaker.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <NuMicro.h>

LOG_MODULE_REGISTER(dma_numaker, CONFIG_DMA_LOG_LEVEL);

#define NUMAKER_PDMA_MAX_TRANSFER_COUNT 16384U
#define NUMAKER_PDMA_MAX_BLOCK_COUNT 16U

struct dma_numaker_pdma_config {
	PDMA_T *pdma;
	const struct reset_dt_spec reset;
	const struct device *clk_dev;
	uint32_t clk_modidx;
	uint32_t clk_src;
	uint32_t clk_div;
	uint8_t channels;
	DSCT_T *desc_table;
	void (*irq_config_func)(const struct device *dev);
};

struct dma_numaker_pdma_channel {
	dma_callback_t cb;
	void *user_data;
	enum dma_channel_direction direction;
	uint8_t dma_slot;
	uint8_t source_addr_adj;
	uint8_t dest_addr_adj;
	uint8_t transfer_width;
	uint8_t block_count;
	uint8_t completed_blocks;
	bool configured;
	bool scatter_enabled;
	bool complete_callback_en;
	bool error_cb_dis;
	uint8_t linked_channel;
	bool source_chaining_en;
	bool dest_chaining_en;
};

struct dma_numaker_pdma_data {
	struct dma_context ctx;
	struct dma_numaker_pdma_channel *channels;
};

static DSCT_T *dma_numaker_pdma_desc(const struct dma_numaker_pdma_config *cfg, uint32_t channel,
				      uint32_t block)
{
	return &cfg->desc_table[(channel * NUMAKER_PDMA_MAX_BLOCK_COUNT) + block];
}

static int dma_numaker_pdma_validate_channel(const struct device *dev, uint32_t channel)
{
	const struct dma_numaker_pdma_config *cfg = dev->config;

	if (channel >= cfg->channels) {
		LOG_ERR("invalid channel %u", channel);
		return -EINVAL;
	}

	return 0;
}

static int dma_numaker_pdma_transfer_width(uint32_t data_size)
{
	switch (data_size) {
	case 1:
	case 2:
	case 4:
		return 0;
	default:
		return -EINVAL;
	}
}

static uint32_t dma_numaker_pdma_width_cfg(uint32_t transfer_width)
{
	return transfer_width == 4U ? PDMA_WIDTH_32 :
		(transfer_width == 2U ? PDMA_WIDTH_16 : PDMA_WIDTH_8);
}

static int dma_numaker_pdma_burst_size(uint32_t burst_len, uint32_t *size)
{
	switch (burst_len) {
	case 0:
	case 1:
		*size = PDMA_BURST_1;
		return 0;
	case 2:
		*size = PDMA_BURST_2;
		return 0;
	case 4:
		*size = PDMA_BURST_4;
		return 0;
	case 8:
		*size = PDMA_BURST_8;
		return 0;
	case 16:
		*size = PDMA_BURST_16;
		return 0;
	case 32:
		*size = PDMA_BURST_32;
		return 0;
	case 64:
		*size = PDMA_BURST_64;
		return 0;
	case 128:
		*size = PDMA_BURST_128;
		return 0;
	default:
		return -ENOTSUP;
	}
}

static int dma_numaker_pdma_addr_adj_ctrl(uint32_t addr_adj, uint32_t inc, uint32_t fix)
{
	switch (addr_adj) {
	case DMA_ADDR_ADJ_INCREMENT:
		return inc;
	case DMA_ADDR_ADJ_NO_CHANGE:
		return fix;
	default:
		return -ENOTSUP;
	}
}

static int dma_numaker_pdma_setup_desc(const struct dma_numaker_pdma_config *cfg,
				       struct dma_numaker_pdma_channel *ch_data,
				       const struct dma_block_config *block_cfg,
				       uint32_t burst_type, uint32_t burst_size,
				       bool callback_en, bool link_next,
				       DSCT_T *desc, DSCT_T *next_desc)
{
	uint32_t trans_count = block_cfg->block_size / ch_data->transfer_width;
	int src_ctrl;
	int dst_ctrl;
	uint32_t ctl;

	if ((block_cfg->block_size == 0U) ||
	    ((block_cfg->block_size % ch_data->transfer_width) != 0U) ||
	    (trans_count > NUMAKER_PDMA_MAX_TRANSFER_COUNT)) {
		return -EINVAL;
	}

	if ((block_cfg->source_gather_interval != 0U) || (block_cfg->dest_scatter_interval != 0U) ||
	    (block_cfg->source_gather_count != 0U) || (block_cfg->dest_scatter_count != 0U) ||
	    block_cfg->source_reload_en || block_cfg->dest_reload_en) {
		return -ENOTSUP;
	}

	src_ctrl = dma_numaker_pdma_addr_adj_ctrl(block_cfg->source_addr_adj, PDMA_SAR_INC,
						   PDMA_SAR_FIX);
	if (src_ctrl < 0) {
		return src_ctrl;
	}

	dst_ctrl = dma_numaker_pdma_addr_adj_ctrl(block_cfg->dest_addr_adj, PDMA_DAR_INC,
						   PDMA_DAR_FIX);
	if (dst_ctrl < 0) {
		return dst_ctrl;
	}

	ctl = ((trans_count - 1U) << PDMA_DSCT_CTL_TXCNT_Pos) |
	      dma_numaker_pdma_width_cfg(ch_data->transfer_width) |
	      (uint32_t)src_ctrl |
	      (uint32_t)dst_ctrl |
	      burst_type |
	      burst_size |
	      (callback_en ? PDMA_TBINTDIS_ENABLE : PDMA_TBINTDIS_DISABLE) |
	      (link_next ? PDMA_OP_SCATTER : PDMA_OP_BASIC);

	desc->CTL = ctl;
	desc->SA = block_cfg->source_address;
	desc->DA = block_cfg->dest_address;
	desc->NEXT = link_next ? ((uint32_t)(uintptr_t)next_desc - cfg->pdma->SCATBA) : 0U;

	return 0;
}

static int dma_numaker_pdma_program(const struct device *dev, uint32_t channel,
				    uint32_t src, uint32_t dst, size_t size)
{
	const struct dma_numaker_pdma_config *cfg = dev->config;
	struct dma_numaker_pdma_data *data = dev->data;
	struct dma_numaker_pdma_channel *ch_data = &data->channels[channel];
	uint32_t src_ctrl = (ch_data->source_addr_adj == DMA_ADDR_ADJ_NO_CHANGE) ?
				    PDMA_SAR_FIX : PDMA_SAR_INC;
	uint32_t dst_ctrl = (ch_data->dest_addr_adj == DMA_ADDR_ADJ_NO_CHANGE) ?
				    PDMA_DAR_FIX : PDMA_DAR_INC;
	uint32_t trans_count = size / ch_data->transfer_width;

	if ((trans_count == 0U) || (trans_count > NUMAKER_PDMA_MAX_TRANSFER_COUNT)) {
		return -EINVAL;
	}

	if ((size % ch_data->transfer_width) != 0U) {
		return -EINVAL;
	}

	PDMA_SetTransferCnt(cfg->pdma, channel, dma_numaker_pdma_width_cfg(ch_data->transfer_width),
				trans_count);
	PDMA_SetTransferAddr(cfg->pdma, channel, src, src_ctrl, dst, dst_ctrl);
	PDMA_SetTransferMode(cfg->pdma, channel,
				(ch_data->direction == MEMORY_TO_MEMORY) ? PDMA_MEM : ch_data->dma_slot,
				0, 0);

	return 0;
}

static int dma_numaker_pdma_setup_scatter(const struct device *dev, uint32_t channel,
					  struct dma_config *dma_cfg,
					  struct dma_numaker_pdma_channel *ch_data,
					  uint32_t burst_type, uint32_t burst_size)
{
	const struct dma_numaker_pdma_config *cfg = dev->config;
	struct dma_block_config *block_cfg = dma_cfg->head_block;
	uint32_t block;
	int err;

	if (dma_cfg->block_count > NUMAKER_PDMA_MAX_BLOCK_COUNT) {
		return -ENOTSUP;
	}

	for (block = 0U; block < dma_cfg->block_count; block++) {
		DSCT_T *desc = dma_numaker_pdma_desc(cfg, channel, block);
		DSCT_T *next_desc = (block + 1U < dma_cfg->block_count) ?
			dma_numaker_pdma_desc(cfg, channel, block + 1U) : NULL;
		bool link_next = (block + 1U) < dma_cfg->block_count;
		bool callback_en = dma_cfg->complete_callback_en || !link_next;

		if (block_cfg == NULL) {
			return -EINVAL;
		}

		err = dma_numaker_pdma_setup_desc(cfg, ch_data, block_cfg, burst_type, burst_size,
						 callback_en, link_next, desc, next_desc);
		if (err != 0) {
			return err;
		}

		if (link_next && (block_cfg->next_block == NULL)) {
			return -EINVAL;
		}

		block_cfg = block_cfg->next_block;
	}

	if (block_cfg != NULL) {
		return -EINVAL;
	}

	PDMA_SetTransferMode(cfg->pdma, channel,
			     (ch_data->direction == MEMORY_TO_MEMORY) ? PDMA_MEM : ch_data->dma_slot,
			     1, (uint32_t)(uintptr_t)dma_numaker_pdma_desc(cfg, channel, 0));

	return 0;
}

static int dma_numaker_pdma_config(const struct device *dev, uint32_t channel,
				   struct dma_config *dma_cfg)
{
	const struct dma_numaker_pdma_config *cfg = dev->config;
	struct dma_numaker_pdma_data *data = dev->data;
	struct dma_numaker_pdma_channel *ch_data;
	uint32_t burst_size;
	uint32_t burst_type;
	int err;

	err = dma_numaker_pdma_validate_channel(dev, channel);
	if (err != 0) {
		return err;
	}

	if ((dma_cfg->block_count == 0U) || (dma_cfg->head_block == NULL)) {
		return -EINVAL;
	}

	if ((dma_cfg->channel_direction != MEMORY_TO_MEMORY) &&
	    (dma_cfg->channel_direction != MEMORY_TO_PERIPHERAL) &&
	    (dma_cfg->channel_direction != PERIPHERAL_TO_MEMORY)) {
		return -ENOTSUP;
	}

	if (dma_cfg->cyclic) {
		return -ENOTSUP;
	}

	if (dma_cfg->source_data_size != dma_cfg->dest_data_size) {
		return -EINVAL;
	}

	err = dma_numaker_pdma_transfer_width(dma_cfg->source_data_size);
	if (err != 0) {
		return err;
	}

	err = dma_numaker_pdma_burst_size(dma_cfg->source_burst_length, &burst_size);
	if (err != 0) {
		return err;
	}

	burst_type = (dma_cfg->source_handshake || dma_cfg->dest_handshake) ?
		PDMA_REQ_SINGLE : PDMA_REQ_BURST;
	ch_data = &data->channels[channel];

	if (PDMA_IS_CH_BUSY(cfg->pdma, channel) != 0U) {
		return -EBUSY;
	}

	ch_data->cb = dma_cfg->dma_callback;
	ch_data->user_data = dma_cfg->user_data;
	ch_data->direction = dma_cfg->channel_direction;
	ch_data->dma_slot = dma_cfg->dma_slot;
	ch_data->source_addr_adj = dma_cfg->head_block->source_addr_adj;
	ch_data->dest_addr_adj = dma_cfg->head_block->dest_addr_adj;
	ch_data->transfer_width = dma_cfg->source_data_size;
	ch_data->block_count = dma_cfg->block_count;
	ch_data->completed_blocks = 0U;
	ch_data->configured = true;
	ch_data->scatter_enabled = (dma_cfg->block_count > 1U);
	ch_data->complete_callback_en = dma_cfg->complete_callback_en;
	ch_data->error_cb_dis = dma_cfg->error_callback_dis;
	ch_data->linked_channel = (uint8_t)dma_cfg->linked_channel;
	ch_data->source_chaining_en = dma_cfg->source_chaining_en;
	ch_data->dest_chaining_en = dma_cfg->dest_chaining_en;

	if (ch_data->scatter_enabled) {
		err = dma_numaker_pdma_setup_scatter(dev, channel, dma_cfg, ch_data, burst_type,
					     burst_size);
	} else {
		if ((dma_cfg->head_block->next_block != NULL) || dma_cfg->head_block->source_gather_en ||
		    dma_cfg->head_block->dest_scatter_en) {
			return -ENOTSUP;
		}

		err = dma_numaker_pdma_setup_desc(cfg, ch_data, dma_cfg->head_block, burst_type,
					  burst_size, true, false,
					  &cfg->pdma->DSCT[channel], NULL);
		if (err == 0) {
			PDMA_SetTransferMode(cfg->pdma, channel,
					     (ch_data->direction == MEMORY_TO_MEMORY) ? PDMA_MEM :
					     ch_data->dma_slot,
					     0, 0);
		}
	}
	if (err != 0) {
		return err;
	}

	if (dma_cfg->dma_callback != NULL) {
		PDMA_EnableInt(cfg->pdma, channel, PDMA_INT_TRANS_DONE);
	} else {
		PDMA_DisableInt(cfg->pdma, channel, PDMA_INT_TRANS_DONE);
	}

	return 0;
}

#ifdef CONFIG_DMA_64BIT
static int dma_numaker_pdma_reload(const struct device *dev, uint32_t channel, uint64_t src,
				   uint64_t dst, size_t size)
#else
static int dma_numaker_pdma_reload(const struct device *dev, uint32_t channel, uint32_t src,
				   uint32_t dst, size_t size)
#endif
{
	const struct dma_numaker_pdma_config *cfg = dev->config;
	struct dma_numaker_pdma_data *data = dev->data;
	int err;

	err = dma_numaker_pdma_validate_channel(dev, channel);
	if (err != 0) {
		return err;
	}

	if (!data->channels[channel].configured) {
		return -EINVAL;
	}

	if (data->channels[channel].scatter_enabled) {
		return -ENOTSUP;
	}

	if (PDMA_IS_CH_BUSY(cfg->pdma, channel) != 0U) {
		return -EBUSY;
	}

	return dma_numaker_pdma_program(dev, channel, (uint32_t)src, (uint32_t)dst, size);
}

static int dma_numaker_pdma_start(const struct device *dev, uint32_t channel)
{
	const struct dma_numaker_pdma_config *cfg = dev->config;
	struct dma_numaker_pdma_data *data = dev->data;
	int err;

	err = dma_numaker_pdma_validate_channel(dev, channel);
	if (err != 0) {
		return err;
	}

	if (!data->channels[channel].configured) {
		return -EINVAL;
	}

	/* Re-enable channel in CHCTL — PDMA_STOP (PAUSE register) clears it */
	cfg->pdma->CHCTL |= BIT(channel);

	PDMA_CLR_TD_FLAG(cfg->pdma, BIT(channel));
	PDMA_CLR_ABORT_FLAG(cfg->pdma, BIT(channel));
	PDMA_CLR_ALIGN_FLAG(cfg->pdma, BIT(channel));

	if (data->channels[channel].direction == MEMORY_TO_MEMORY) {
		PDMA_Trigger(cfg->pdma, channel);
	}

	return 0;
}

static int dma_numaker_pdma_stop(const struct device *dev, uint32_t channel)
{
	const struct dma_numaker_pdma_config *cfg = dev->config;
	int err;

	err = dma_numaker_pdma_validate_channel(dev, channel);
	if (err != 0) {
		return err;
	}

	PDMA_STOP(cfg->pdma, channel);
	PDMA_CLR_TD_FLAG(cfg->pdma, BIT(channel));
	PDMA_CLR_ABORT_FLAG(cfg->pdma, BIT(channel));
	PDMA_CLR_ALIGN_FLAG(cfg->pdma, BIT(channel));

	return 0;
}

static int dma_numaker_pdma_get_status(const struct device *dev, uint32_t channel,
				       struct dma_status *status)
{
	const struct dma_numaker_pdma_config *cfg = dev->config;
	struct dma_numaker_pdma_data *data = dev->data;
	uint32_t ctl;
	int err;

	err = dma_numaker_pdma_validate_channel(dev, channel);
	if (err != 0) {
		return err;
	}

	status->busy = (PDMA_IS_CH_BUSY(cfg->pdma, channel) != 0U);
	status->dir = data->channels[channel].direction;
	status->pending_length = 0U;

	if (status->busy) {
		if (data->channels[channel].scatter_enabled) {
			ctl = cfg->pdma->CURSCAT[channel] != 0U ?
				((DSCT_T *)(uintptr_t)cfg->pdma->CURSCAT[channel])->CTL :
				cfg->pdma->DSCT[channel].CTL;
		} else {
			ctl = cfg->pdma->DSCT[channel].CTL;
		}
		status->pending_length =
			(((ctl & PDMA_DSCT_CTL_TXCNT_Msk) >> PDMA_DSCT_CTL_TXCNT_Pos) + 1U) *
			data->channels[channel].transfer_width;
	}

	return 0;
}

static int dma_numaker_pdma_get_attribute(const struct device *dev, uint32_t type,
					  uint32_t *value)
{
	ARG_UNUSED(dev);

	switch (type) {
	case DMA_ATTR_BUFFER_ADDRESS_ALIGNMENT:
	case DMA_ATTR_BUFFER_SIZE_ALIGNMENT:
	case DMA_ATTR_COPY_ALIGNMENT:
		*value = 1U;
		return 0;
	case DMA_ATTR_MAX_BLOCK_COUNT:
		*value = NUMAKER_PDMA_MAX_BLOCK_COUNT;
		return 0;
	default:
		return -EINVAL;
	}
}

static bool dma_numaker_pdma_chan_filter(const struct device *dev, int channel,
					 void *filter_param)
{
	const struct dma_numaker_pdma_config *cfg = dev->config;
	uint32_t filter;

	if ((channel < 0) || (channel >= cfg->channels)) {
		return false;
	}

	if (filter_param == NULL) {
		return true;
	}

	filter = *((uint32_t *)filter_param);

	return ((filter & BIT(channel)) != 0U);
}

static void dma_numaker_pdma_complete_cb(const struct device *dev, uint32_t flags, int status)
{
	struct dma_numaker_pdma_data *data = dev->data;
	const struct dma_numaker_pdma_config *cfg = dev->config;
	uint32_t link_mask = 0U;
	uint32_t processed = flags;

	while (flags != 0U) {
		uint32_t channel = find_lsb_set(flags) - 1U;
		struct dma_numaker_pdma_channel *ch_data = &data->channels[channel];
		int cb_status = status;

		flags &= ~BIT(channel);

		if ((status >= 0) && ch_data->scatter_enabled && ch_data->complete_callback_en &&
		    (ch_data->completed_blocks < ch_data->block_count)) {
			ch_data->completed_blocks++;
			cb_status = (ch_data->completed_blocks < ch_data->block_count) ?
				DMA_STATUS_BLOCK : DMA_STATUS_COMPLETE;
		} else if ((status >= 0) && ch_data->scatter_enabled) {
			ch_data->completed_blocks = ch_data->block_count;
		}

		if ((ch_data->cb != NULL) && (cb_status >= 0 || !ch_data->error_cb_dis)) {
			ch_data->cb(dev, ch_data->user_data, channel, cb_status);
		}

		if ((status >= 0) && (ch_data->source_chaining_en || ch_data->dest_chaining_en)) {
			link_mask |= BIT(ch_data->linked_channel);
		}
	}

	/*
	 * Only clear the flags we actually processed. Re-reading
	 * PDMA_GET_TD_STS here would race with newly completed
	 * channels and silently discard their completion.
	 */
	if (status >= 0) {
		PDMA_CLR_TD_FLAG(cfg->pdma, processed);
	} else {
		PDMA_CLR_ABORT_FLAG(cfg->pdma, processed);
		PDMA_CLR_ALIGN_FLAG(cfg->pdma, processed);
	}

	while (link_mask != 0U) {
		uint32_t linked_ch = find_lsb_set(link_mask) - 1U;

		link_mask &= ~BIT(linked_ch);
		PDMA_CLR_TD_FLAG(cfg->pdma, BIT(linked_ch));
		PDMA_CLR_ABORT_FLAG(cfg->pdma, BIT(linked_ch));
		PDMA_Trigger(cfg->pdma, linked_ch);
	}
}

static void dma_numaker_pdma_isr(const struct device *dev)
{
	const struct dma_numaker_pdma_config *cfg = dev->config;
	uint32_t intsts = PDMA_GET_INT_STATUS(cfg->pdma);

	if ((intsts & PDMA_INTSTS_ABTIF_Msk) != 0U) {
		dma_numaker_pdma_complete_cb(dev, PDMA_GET_ABORT_STS(cfg->pdma), -EIO);
	}

	if ((intsts & PDMA_INTSTS_TDIF_Msk) != 0U) {
		dma_numaker_pdma_complete_cb(dev, PDMA_GET_TD_STS(cfg->pdma), DMA_STATUS_COMPLETE);
	}

	if ((intsts & PDMA_INTSTS_ALIGNF_Msk) != 0U) {
		dma_numaker_pdma_complete_cb(dev, PDMA_GET_ALIGN_STS(cfg->pdma), -EIO);
	}
}

static int dma_numaker_pdma_init(const struct device *dev)
{
	const struct dma_numaker_pdma_config *cfg = dev->config;
	struct numaker_scc_subsys scc_subsys;
	int err;

	SYS_UnlockReg();

	memset(&scc_subsys, 0, sizeof(scc_subsys));
	scc_subsys.subsys_id = NUMAKER_SCC_SUBSYS_ID_PCC;
	scc_subsys.pcc.clk_modidx = cfg->clk_modidx;
	scc_subsys.pcc.clk_src = cfg->clk_src;
	scc_subsys.pcc.clk_div = cfg->clk_div;

	err = clock_control_on(cfg->clk_dev, (clock_control_subsys_t)&scc_subsys);
	if (err != 0) {
		goto out;
	}

	err = clock_control_configure(cfg->clk_dev, (clock_control_subsys_t)&scc_subsys, NULL);
	if (err != 0) {
		goto out;
	}

	if (!device_is_ready(cfg->reset.dev)) {
		err = -ENODEV;
		goto out;
	}

	(void)reset_line_toggle_dt(&cfg->reset);

	PDMA_Open(cfg->pdma, BIT_MASK(cfg->channels));
	cfg->pdma->SCATBA = (uint32_t)(uintptr_t)cfg->desc_table;
	cfg->irq_config_func(dev);

out:
	SYS_LockReg();
	return err;
}

static DEVICE_API(dma, dma_numaker_pdma_driver_api) = {
	.config = dma_numaker_pdma_config,
	.reload = dma_numaker_pdma_reload,
	.start = dma_numaker_pdma_start,
	.stop = dma_numaker_pdma_stop,
	.get_status = dma_numaker_pdma_get_status,
	.get_attribute = dma_numaker_pdma_get_attribute,
	.chan_filter = dma_numaker_pdma_chan_filter,
};

#define DMA_NUMAKER_PDMA_IRQ_CONFIG(inst)                                                           \
	static void dma_numaker_pdma_irq_config_##inst(const struct device *dev)                    \
	{                                                                                             \
		ARG_UNUSED(dev);                                                                        \
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority),                         \
			    dma_numaker_pdma_isr, DEVICE_DT_INST_GET(inst), 0);                          \
		irq_enable(DT_INST_IRQN(inst));                                                       \
	}

#define DMA_NUMAKER_PDMA_INIT(inst)                                                                 \
	DMA_NUMAKER_PDMA_IRQ_CONFIG(inst);                                                            \
	static DSCT_T dma_numaker_pdma_desc_table_##inst[                                          \
		DT_INST_PROP(inst, dma_channels) * NUMAKER_PDMA_MAX_BLOCK_COUNT] __aligned(4);      \
	static const struct dma_numaker_pdma_config dma_numaker_pdma_config_##inst = {               \
		.pdma = (PDMA_T *)DT_INST_REG_ADDR(inst),                                              \
		.reset = RESET_DT_SPEC_INST_GET(inst),                                                 \
		.clk_dev = DEVICE_DT_GET(DT_PARENT(DT_INST_CLOCKS_CTLR(inst))),                        \
		.clk_modidx = DT_INST_CLOCKS_CELL(inst, clock_module_index),                           \
		.clk_src = DT_INST_CLOCKS_CELL(inst, clock_source),                                    \
		.clk_div = DT_INST_CLOCKS_CELL(inst, clock_divider),                                   \
		.channels = DT_INST_PROP(inst, dma_channels),                                          \
		.desc_table = dma_numaker_pdma_desc_table_##inst,                                      \
		.irq_config_func = dma_numaker_pdma_irq_config_##inst,                                 \
	};                                                                                            \
	static struct dma_numaker_pdma_channel dma_numaker_pdma_channels_##inst[                     \
		DT_INST_PROP(inst, dma_channels)];                                                   \
	ATOMIC_DEFINE(dma_numaker_pdma_atomic_##inst, DT_INST_PROP(inst, dma_channels));             \
	static struct dma_numaker_pdma_data dma_numaker_pdma_data_##inst = {                         \
		.ctx = {                                                                              \
			.magic = DMA_MAGIC,                                                             \
			.dma_channels = DT_INST_PROP(inst, dma_channels),                              \
			.atomic = dma_numaker_pdma_atomic_##inst,                                      \
		},                                                                                     \
		.channels = dma_numaker_pdma_channels_##inst,                                         \
	};                                                                                            \
	DEVICE_DT_INST_DEFINE(inst, dma_numaker_pdma_init, NULL, &dma_numaker_pdma_data_##inst,      \
			      &dma_numaker_pdma_config_##inst, PRE_KERNEL_1,                        \
			      CONFIG_DMA_INIT_PRIORITY, &dma_numaker_pdma_driver_api)

DT_INST_FOREACH_STATUS_OKAY(DMA_NUMAKER_PDMA_INIT)
