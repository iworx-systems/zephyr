/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2023 Nuvoton Technology Corporation.
 */

#define DT_DRV_COMPAT nuvoton_numaker_spi

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/clock_control_numaker.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/spi/rtio.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(spi_numaker, CONFIG_SPI_LOG_LEVEL);

#include "spi_context.h"
#include <NuMicro.h>

#define SPI_NUMAKER_TX_NOP 0x00
#define SPI_NUMAKER_DMA_TIMEOUT_MS 1000
#define SPI_NUMAKER_DMA_TX_DONE BIT(0)
#define SPI_NUMAKER_DMA_RX_DONE BIT(1)
#define SPI_NUMAKER_CTL_MSK (SPI_CTL_DWIDTH_Msk | SPI_CTL_SLAVE_Msk | \
			     SPI_CTL_CLKPOL_Msk | SPI_CTL_RXNEG_Msk | \
			     SPI_CTL_TXNEG_Msk)

struct spi_numaker_config {
	SPI_T *spi;
	bool is_qspi;
	bool dma_enabled;
	const struct reset_dt_spec reset;
	/* clock configuration */
	uint32_t clk_modidx;
	uint32_t clk_src;
	uint32_t clk_div;
	const struct device *clk_dev;
	const struct pinctrl_dev_config *pincfg;
#ifdef CONFIG_SPI_NUMAKER_DMA
	const struct device *dma_dev;
	uint32_t tx_dma_channel;
	uint32_t tx_dma_slot;
	uint32_t rx_dma_channel;
	uint32_t rx_dma_slot;
#endif
};

struct spi_numaker_data {
	struct spi_context ctx;
#ifdef CONFIG_SPI_NUMAKER_DMA
	struct k_sem dma_sync;
	uint8_t dma_done;
	int dma_status;
	uint32_t dma_dummy_tx;
	uint32_t dma_dummy_rx;
#endif
};

/*
 * CPOL/CPHA = 0/0 --> SPI_MODE_0
 * CPOL/CPHA = 0/1 --> SPI_MODE_1
 * CPOL/CPHA = 1/0 --> SPI_MODE_2
 * CPOL/CPHA = 1/1 --> SPI_MODE_3
 */
static const uint32_t smode_tbl[4] = {
	SPI_MODE_0, SPI_MODE_1, SPI_MODE_2, SPI_MODE_3
};

static const uint32_t qsmode_tbl[4] = {
	QSPI_MODE_0, QSPI_MODE_1, QSPI_MODE_2, QSPI_MODE_3
};

static int spi_numaker_configure(const struct device *dev, const struct spi_config *config)
{
	int mode;
	struct spi_numaker_data *data = dev->data;
	const struct spi_numaker_config *dev_cfg = dev->config;

	LOG_DBG("%s", __func__);
	if (spi_context_configured(&data->ctx, config)) {
		return 0;
	}

	if (SPI_MODE_GET(config->operation) & SPI_MODE_LOOP) {
		LOG_ERR("Loop back mode not support");
		return -ENOTSUP;
	}

	if (SPI_OP_MODE_GET(config->operation) == SPI_OP_MODE_SLAVE) {
		LOG_ERR("Slave mode not support");
		return -ENOTSUP;
	}

	/* Clear FIFO */
	SPI_ClearRxFIFO(dev_cfg->spi);
	SPI_ClearTxFIFO(dev_cfg->spi);

	if (SPI_MODE_GET(config->operation) & SPI_MODE_CPOL) {
		mode = (SPI_MODE_GET(config->operation) & SPI_MODE_CPHA) ? 3 : 2;
	} else {
		mode = (SPI_MODE_GET(config->operation) & SPI_MODE_CPHA) ? 1 : 0;
	}

	/* Make SPI module be ready to transfer */
	if (dev_cfg->is_qspi) {
		QSPI_Open((QSPI_T *)dev_cfg->spi,
			  (SPI_OP_MODE_GET(config->operation) == SPI_OP_MODE_SLAVE) ? QSPI_SLAVE
										    : QSPI_MASTER,
			  qsmode_tbl[mode],
			  SPI_WORD_SIZE_GET(config->operation), config->frequency);
	} else {
		/* Clear SPI CTL before set */
		dev_cfg->spi->CTL &= ~SPI_NUMAKER_CTL_MSK;
		SPI_Open(dev_cfg->spi,
			 (SPI_OP_MODE_GET(config->operation) == SPI_OP_MODE_SLAVE) ? SPI_SLAVE
										   : SPI_MASTER,
			 smode_tbl[mode],
			 SPI_WORD_SIZE_GET(config->operation), config->frequency);
	}

	/* Set Transfer LSB or MSB first */
	if ((config->operation) & SPI_TRANSFER_LSB) {
		SPI_SET_LSB_FIRST(dev_cfg->spi);
	} else {
		SPI_SET_MSB_FIRST(dev_cfg->spi);
	}

	/* full/half duplex */
	if (config->operation & SPI_HALF_DUPLEX) {
		/* Not implemented, though it is possible in hardware */
		return -ENOTSUP;
	}

	/* Active high CS logic */
	if (config->operation & SPI_CS_ACTIVE_HIGH) {
		SPI_SET_SS_HIGH(dev_cfg->spi);
	} else {
		SPI_SET_SS_LOW(dev_cfg->spi);
	}

	/* Enable the automatic hardware slave select function. Select the SS pin and configure as
	 * low-active.
	 */
	if (UTIL_OR(IS_ENABLED(DT_SPI_CTX_HAS_NO_CS_GPIOS), (data->ctx.num_cs_gpios == 0))) {
		SPI_EnableAutoSS(dev_cfg->spi, SPI_SS, SPI_SS_ACTIVE_LOW);
	} else {
		SPI_DisableAutoSS(dev_cfg->spi);
	}

	/* Be able to set TX/RX FIFO threshold, for ex: SPI_SetFIFO(dev_cfg->spi, 2, 2) */

	data->ctx.config = config;

	return 0;
}

static int spi_numaker_txrx(const struct device *dev, uint8_t spi_dfs)
{
	struct spi_numaker_data *data = dev->data;
	const struct spi_numaker_config *dev_cfg = dev->config;
	struct spi_context *ctx = &data->ctx;
	uint32_t tx_frame, rx_frame;
	uint32_t time_out_cnt;

	LOG_DBG("%s", __func__);

	if (spi_context_tx_on(ctx)) {
		tx_frame = ((ctx->tx_buf == NULL) ? SPI_NUMAKER_TX_NOP
			   : UNALIGNED_GET((uint32_t *)data->ctx.tx_buf));
		/* Write to TX register */
		SPI_WRITE_TX(dev_cfg->spi, tx_frame);
		spi_context_update_tx(ctx, spi_dfs, 1);
		LOG_DBG("%s --> TX [0x%x] done", __func__, tx_frame);
	} else {
		/* Write dummy data to TX register */
		SPI_WRITE_TX(dev_cfg->spi, 0x00U);
	}

	/* Read received data */
	if (spi_context_rx_on(ctx)) {
		if (SPI_GET_RX_FIFO_COUNT(dev_cfg->spi) > 0) {
			rx_frame = SPI_READ_RX(dev_cfg->spi);
			if (ctx->rx_buf != NULL) {
				if (spi_dfs > 2) {
					UNALIGNED_PUT(rx_frame,
					(uint32_t *)data->ctx.rx_buf);
				} else if (spi_dfs > 1) {
					UNALIGNED_PUT(rx_frame,
					(uint16_t *)data->ctx.rx_buf);
				} else {
					UNALIGNED_PUT(rx_frame,
					(uint8_t *)data->ctx.rx_buf);
				}
			}
			spi_context_update_rx(ctx, spi_dfs, 1);
			LOG_DBG("%s --> RX [0x%x] done", __func__, rx_frame);
		}
	}

	/* Check SPI busy status */
	time_out_cnt = SystemCoreClock; /* 1 second time-out */
	while (SPI_IS_BUSY(dev_cfg->spi)) {
		if (--time_out_cnt == 0) {
			LOG_ERR("Wait for SPI time-out");
			return -EIO;
		}
	}

	LOG_DBG("%s --> exit", __func__);
	return 0;
}

/* Remain TX/RX Data in spi_context TX/RX buffer */
static bool spi_numaker_remain_words(struct spi_numaker_data *data)
{
	return spi_context_tx_on(&data->ctx) || spi_context_rx_on(&data->ctx);
}

#ifdef CONFIG_SPI_NUMAKER_DMA
static bool spi_numaker_use_dma(const struct device *dev, uint8_t spi_dfs)
{
	const struct spi_numaker_config *dev_cfg = dev->config;

	/* 24-bit frame width is not natively handled in PDMA path. */
	return dev_cfg->dma_enabled && (spi_dfs != 3U);
}

static void spi_numaker_dma_cb(const struct device *dma_dev, void *arg, uint32_t channel, int status)
{
	const struct device *dev = arg;
	const struct spi_numaker_config *dev_cfg = dev->config;
	struct spi_numaker_data *data = dev->data;

	ARG_UNUSED(dma_dev);

	if (status < 0) {
		LOG_ERR("DMA callback error status=%d channel=%u", status, channel);
		data->dma_status = status;
		k_sem_give(&data->dma_sync);
		return;
	}

	if (channel == dev_cfg->tx_dma_channel) {
		LOG_DBG("DMA TX complete");
		data->dma_done |= SPI_NUMAKER_DMA_TX_DONE;
	} else if (channel == dev_cfg->rx_dma_channel) {
		LOG_DBG("DMA RX complete");
		data->dma_done |= SPI_NUMAKER_DMA_RX_DONE;
	} else {
		LOG_ERR("DMA callback unknown channel=%u", channel);
		data->dma_status = -EIO;
	}

	k_sem_give(&data->dma_sync);
}

static int spi_numaker_dma_config(const struct device *dev, uint32_t channel, uint32_t slot,
				  enum dma_channel_direction direction, uint32_t src_addr,
				  uint32_t dst_addr, uint32_t size, uint8_t spi_dfs,
				  uint32_t src_adj, uint32_t dst_adj)
{
	const struct spi_numaker_config *dev_cfg = dev->config;
	struct dma_config dma_cfg = { 0 };
	struct dma_block_config blk_cfg = { 0 };

	dma_cfg.channel_direction = direction;
	dma_cfg.source_data_size = spi_dfs;
	dma_cfg.dest_data_size = spi_dfs;
	dma_cfg.source_burst_length = 1U;
	dma_cfg.dest_burst_length = 1U;
	/* SPI peripheral requests should use single-request handshake mode. */
	dma_cfg.source_handshake = (direction == PERIPHERAL_TO_MEMORY) ? 1U : 0U;
	dma_cfg.dest_handshake = (direction == MEMORY_TO_PERIPHERAL) ? 1U : 0U;
	dma_cfg.dma_slot = slot;
	dma_cfg.dma_callback = spi_numaker_dma_cb;
	dma_cfg.user_data = (void *)dev;
	dma_cfg.complete_callback_en = 1U;
	dma_cfg.error_callback_dis = 0U;
	dma_cfg.block_count = 1U;
	dma_cfg.head_block = &blk_cfg;

	blk_cfg.block_size = size;
	blk_cfg.source_address = src_addr;
	blk_cfg.dest_address = dst_addr;
	blk_cfg.source_addr_adj = src_adj;
	blk_cfg.dest_addr_adj = dst_adj;

	return dma_config(dev_cfg->dma_dev, channel, &dma_cfg);
}

static int spi_numaker_txrx_dma(const struct device *dev, uint8_t spi_dfs)
{
	struct spi_numaker_data *data = dev->data;
	const struct spi_numaker_config *dev_cfg = dev->config;
	struct spi_context *ctx = &data->ctx;
	int ret;
	uint32_t time_out_cnt;

	LOG_DBG("DMA: spi_dfs=%d, rx_enabled=%d, tx_enabled=%d", spi_dfs,
		spi_context_rx_buf_on(ctx), spi_context_tx_buf_on(ctx));

	SPI_DISABLE_RX_PDMA(dev_cfg->spi);
	SPI_DISABLE_TX_PDMA(dev_cfg->spi);
	dev_cfg->spi->PDMACTL |= SPI_PDMACTL_PDMARST_Msk;
	SPI_ClearRxFIFO(dev_cfg->spi);
	SPI_ClearTxFIFO(dev_cfg->spi);

	while (spi_numaker_remain_words(data)) {
		size_t chunk_frames = spi_context_max_continuous_chunk(ctx);
		size_t chunk_bytes = chunk_frames * spi_dfs;
		uint32_t tx_addr;
		uint32_t rx_addr;

		if (chunk_frames == 0U) {
			return -EINVAL;
		}

		tx_addr = spi_context_tx_buf_on(ctx) ? (uint32_t)(uintptr_t)ctx->tx_buf :
			(uint32_t)(uintptr_t)&data->dma_dummy_tx;
		rx_addr = spi_context_rx_buf_on(ctx) ? (uint32_t)(uintptr_t)ctx->rx_buf :
			(uint32_t)(uintptr_t)&data->dma_dummy_rx;

		LOG_DBG("DMA: chunk_frames=%zu chunk_bytes=%zu tx_addr=0x%x rx_addr=0x%x tx_use_real=%d rx_use_real=%d",
			chunk_frames, chunk_bytes, tx_addr, rx_addr,
			spi_context_tx_buf_on(ctx), spi_context_rx_buf_on(ctx));

		ret = spi_numaker_dma_config(dev, dev_cfg->tx_dma_channel, dev_cfg->tx_dma_slot,
					     MEMORY_TO_PERIPHERAL, tx_addr,
					     (uint32_t)(uintptr_t)&dev_cfg->spi->TX,
					     chunk_bytes, spi_dfs,
					     spi_context_tx_buf_on(ctx) ? DMA_ADDR_ADJ_INCREMENT :
					     DMA_ADDR_ADJ_NO_CHANGE,
					     DMA_ADDR_ADJ_NO_CHANGE);
		if (ret < 0) {
			return ret;
		}

		ret = spi_numaker_dma_config(dev, dev_cfg->rx_dma_channel, dev_cfg->rx_dma_slot,
					     PERIPHERAL_TO_MEMORY,
					     (uint32_t)(uintptr_t)&dev_cfg->spi->RX,
					     rx_addr, chunk_bytes, spi_dfs,
					     DMA_ADDR_ADJ_NO_CHANGE,
					     spi_context_rx_buf_on(ctx) ? DMA_ADDR_ADJ_INCREMENT :
					     DMA_ADDR_ADJ_NO_CHANGE);
		if (ret < 0) {
			return ret;
		}

		data->dma_done = 0U;
		data->dma_status = 0;
		k_sem_reset(&data->dma_sync);

		ret = dma_start(dev_cfg->dma_dev, dev_cfg->rx_dma_channel);
		if (ret < 0) {
			return ret;
		}

		ret = dma_start(dev_cfg->dma_dev, dev_cfg->tx_dma_channel);
		if (ret < 0) {
			(void)dma_stop(dev_cfg->dma_dev, dev_cfg->rx_dma_channel);
			return ret;
		}

		SPI_TRIGGER_RX_PDMA(dev_cfg->spi);
		SPI_TRIGGER_TX_PDMA(dev_cfg->spi);

		while ((data->dma_done != (SPI_NUMAKER_DMA_TX_DONE | SPI_NUMAKER_DMA_RX_DONE)) &&
		       (data->dma_status == 0)) {
			ret = k_sem_take(&data->dma_sync, K_MSEC(SPI_NUMAKER_DMA_TIMEOUT_MS));
			if (ret < 0) {
				LOG_ERR("SPI DMA timeout: done=0x%x status=%d",
					data->dma_done, data->dma_status);
				data->dma_status = -ETIMEDOUT;
				break;
			}
		}

		SPI_DISABLE_RX_PDMA(dev_cfg->spi);
		SPI_DISABLE_TX_PDMA(dev_cfg->spi);
		dev_cfg->spi->PDMACTL |= SPI_PDMACTL_PDMARST_Msk;
		(void)dma_stop(dev_cfg->dma_dev, dev_cfg->tx_dma_channel);
		(void)dma_stop(dev_cfg->dma_dev, dev_cfg->rx_dma_channel);

		time_out_cnt = SystemCoreClock;
		while (SPI_IS_BUSY(dev_cfg->spi)) {
			if (--time_out_cnt == 0U) {
				return -EIO;
			}
		}

		if (data->dma_status < 0) {
			SPI_ClearRxFIFO(dev_cfg->spi);
			SPI_ClearTxFIFO(dev_cfg->spi);
			return data->dma_status;
		}

		spi_context_update_tx(ctx, spi_dfs, chunk_frames);
		spi_context_update_rx(ctx, spi_dfs, chunk_frames);
	}

	SPI_ClearRxFIFO(dev_cfg->spi);
	SPI_ClearTxFIFO(dev_cfg->spi);

	return 0;
}
#endif

static int spi_numaker_transceive(const struct device *dev, const struct spi_config *config,
				  const struct spi_buf_set *tx_bufs,
				  const struct spi_buf_set *rx_bufs)
{
	struct spi_numaker_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;
	const struct spi_numaker_config *dev_cfg = dev->config;
	int ret;
	uint8_t word_size, spi_dfs;

	LOG_DBG("%s", __func__);
	spi_context_lock(ctx, false, NULL, NULL, config);

	ret = spi_numaker_configure(dev, config);
	if (ret < 0) {
		goto done;
	}

	word_size = SPI_WORD_SIZE_GET(ctx->config->operation);

	switch (word_size) {
	case 8:
		spi_dfs = 1;
		break;
	case 16:
		spi_dfs = 2;
		break;
	case 24:
		spi_dfs = 3;
		break;
	case 32:
		spi_dfs = 4;
		break;
	default:
		spi_dfs = 0;
		LOG_ERR("Not support SPI WORD size as [%d] bits", word_size);
		ret = -ENOTSUP;
		goto done;
	}

	LOG_DBG("%s -->word_size [%d]", __func__, word_size);

	SPI_ENABLE(dev_cfg->spi);

	spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, spi_dfs);

	/* if cs is defined: software cs control, set active true */
	spi_context_cs_control(&data->ctx, true);

	/* transceive tx/rx data */
	#ifdef CONFIG_SPI_NUMAKER_DMA
	if (spi_numaker_use_dma(dev, spi_dfs)) {
		LOG_DBG("Using DMA path");
		ret = spi_numaker_txrx_dma(dev, spi_dfs);
	} else
	#endif
	{
		LOG_DBG("Using polling path");
		do {
			ret = spi_numaker_txrx(dev, spi_dfs);
			if (ret < 0) {
				break;
			}
		} while (spi_numaker_remain_words(data));
	}

	/* if cs is defined: software cs control, set active false */
	spi_context_cs_control(&data->ctx, false);
	SPI_DISABLE(dev_cfg->spi);

done:
	spi_context_release(ctx, ret);
	LOG_DBG("%s --> [%d]", __func__, ret);
	return ret;
}

static int spi_numaker_release(const struct device *dev, const struct spi_config *config)
{
	struct spi_numaker_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;

	if (!spi_context_configured(ctx, config)) {
		return -EINVAL;
	}
	spi_context_unlock_unconditionally(ctx);

	return 0;
}

static DEVICE_API(spi, spi_numaker_driver_api) = {
	.transceive = spi_numaker_transceive,
#ifdef CONFIG_SPI_RTIO
	.iodev_submit = spi_rtio_iodev_default_submit,
#endif
	.release = spi_numaker_release
};

static int spi_numaker_init(const struct device *dev)
{
	struct spi_numaker_data *data = dev->data;
	const struct spi_numaker_config *dev_cfg = dev->config;
	int err = 0;
	struct numaker_scc_subsys scc_subsys;

	SYS_UnlockReg();

	/* CLK controller */
	memset(&scc_subsys, 0x00, sizeof(scc_subsys));
	scc_subsys.subsys_id = NUMAKER_SCC_SUBSYS_ID_PCC;
	scc_subsys.pcc.clk_modidx = dev_cfg->clk_modidx;
	scc_subsys.pcc.clk_src = dev_cfg->clk_src;
	scc_subsys.pcc.clk_div = dev_cfg->clk_div;

	/* Equivalent to CLK_EnableModuleClock() */
	err = clock_control_on(dev_cfg->clk_dev, (clock_control_subsys_t)&scc_subsys);
	if (err != 0) {
		goto done;
	}
	/* Equivalent to CLK_SetModuleClock() */
	err = clock_control_configure(dev_cfg->clk_dev, (clock_control_subsys_t)&scc_subsys, NULL);
	if (err != 0) {
		goto done;
	}

	err = pinctrl_apply_state(dev_cfg->pincfg, PINCTRL_STATE_DEFAULT);
	if (err) {
		LOG_ERR("Failed to apply pinctrl state");
		goto done;
	}

	err = spi_context_cs_configure_all(&data->ctx);
	if (err < 0) {
		goto done;
	}

#ifdef CONFIG_SPI_NUMAKER_DMA
	if (dev_cfg->dma_enabled) {
		if (!device_is_ready(dev_cfg->dma_dev)) {
			LOG_ERR("dma controller device is not ready");
			err = -ENODEV;
			goto done;
		}

		k_sem_init(&data->dma_sync, 0, 2);
		data->dma_dummy_tx = SPI_NUMAKER_TX_NOP;
		data->dma_dummy_rx = 0U;
	}
#endif

	spi_context_unlock_unconditionally(&data->ctx);

	/* Reset this module, same as BSP's SYS_ResetModule(id_rst) */
	if (!device_is_ready(dev_cfg->reset.dev)) {
		LOG_ERR("reset controller not ready");
		err = -ENODEV;
		goto done;
	}

	/* Reset SPI to default state */
	reset_line_toggle_dt(&dev_cfg->reset);

done:
	SYS_LockReg();
	return err;
}

#define NUMAKER_SPI_INIT(inst)                                                                     \
	PINCTRL_DT_INST_DEFINE(inst);                                                              \
	static struct spi_numaker_data spi_numaker_data_##inst = {                                 \
		SPI_CONTEXT_INIT_LOCK(spi_numaker_data_##inst, ctx),                               \
		SPI_CONTEXT_INIT_SYNC(spi_numaker_data_##inst, ctx),                               \
		SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(inst), ctx)};                          \
	static struct spi_numaker_config spi_numaker_config_##inst = {                             \
		.spi = (SPI_T *)DT_INST_REG_ADDR(inst),                                            \
		.is_qspi = DT_INST_NODE_HAS_PROP(inst, qspi),                                      \
		.dma_enabled = DT_INST_DMAS_HAS_NAME(inst, tx) && DT_INST_DMAS_HAS_NAME(inst, rx),\
		.reset = RESET_DT_SPEC_INST_GET(inst),                                             \
		.clk_modidx = DT_INST_CLOCKS_CELL(inst, clock_module_index),                       \
		.clk_src = DT_INST_CLOCKS_CELL(inst, clock_source),                                \
		.clk_div = DT_INST_CLOCKS_CELL(inst, clock_divider),                               \
		.clk_dev = DEVICE_DT_GET(DT_PARENT(DT_INST_CLOCKS_CTLR(inst))),                    \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                    \
		IF_ENABLED(CONFIG_SPI_NUMAKER_DMA, (                                               \
			.dma_dev = COND_CODE_1(DT_INST_DMAS_HAS_NAME(inst, tx),                      \
				(DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(inst, tx))), (NULL)),        \
			.tx_dma_channel = COND_CODE_1(DT_INST_DMAS_HAS_NAME(inst, tx),               \
				(DT_INST_DMAS_CELL_BY_NAME(inst, tx, channel)), (0)),                \
			.tx_dma_slot = COND_CODE_1(DT_INST_DMAS_HAS_NAME(inst, tx),                  \
				(DT_INST_DMAS_CELL_BY_NAME(inst, tx, slot)), (0)),                   \
			.rx_dma_channel = COND_CODE_1(DT_INST_DMAS_HAS_NAME(inst, rx),               \
				(DT_INST_DMAS_CELL_BY_NAME(inst, rx, channel)), (0)),                \
			.rx_dma_slot = COND_CODE_1(DT_INST_DMAS_HAS_NAME(inst, rx),                  \
				(DT_INST_DMAS_CELL_BY_NAME(inst, rx, slot)), (0)),                   \
		))                                                                                 \
	};                                                                                         \
	SPI_DEVICE_DT_INST_DEFINE(inst, spi_numaker_init, NULL, &spi_numaker_data_##inst,          \
			      &spi_numaker_config_##inst, POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,   \
			      &spi_numaker_driver_api);

DT_INST_FOREACH_STATUS_OKAY(NUMAKER_SPI_INIT)
