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
#ifdef CONFIG_SPI_ASYNC
	void (*irq_config_func)(const struct device *dev);
#endif
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
	bool dma_cfg_cached;
	uint32_t dma_tx_src_adj;
	uint32_t dma_rx_dst_adj;
#ifdef CONFIG_SPI_ASYNC
	bool async_dma;    /* true when an async DMA transfer is in progress */
#endif
#endif
#ifdef CONFIG_SPI_ASYNC
	uint8_t spi_dfs;       /* frame size in bytes, saved for async callback */
	const struct device *dev; /* back-pointer for async work handler */
	struct k_work async_work; /* work item for non-DMA async path (fallback) */
	uint8_t async_irq_batch; /* frames in current FIFO batch for ISR */
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

	/* Enable automatic hardware slave select when the device uses native
	 * (non-GPIO) chip select.  Derive the active level from the per-device
	 * SPI_CS_ACTIVE_HIGH operation bit so it matches the devicetree.
	 * Skip manual SPI_SET_SS_HIGH/LOW for auto-SS — SPI_EnableAutoSS
	 * configures everything atomically and avoids glitching the SS pin.
	 */
	if (!spi_cs_is_gpio(config)) {
		uint32_t ss_level = (config->operation & SPI_CS_ACTIVE_HIGH)
			? SPI_SS_ACTIVE_HIGH : SPI_SS_ACTIVE_LOW;
		SPI_EnableAutoSS(dev_cfg->spi, SPI_SS, ss_level);
		LOG_WRN("Enable");
	} else {
		/* Disable auto-SS so the hardware SS pin is not driven during
		 * transfers to GPIO-CS devices.  SPI_DisableAutoSS clears SS
		 * and AUTOSS while preserving SSACTPOL, so the pin stays at
		 * its inactive level (set by the last SPI_EnableAutoSS call).
		 * Do NOT call SPI_SET_SS_HIGH/LOW here — they set SS=1 and
		 * change SSACTPOL, which would actively drive the hardware SS
		 * pin and cause crosstalk with the auto-SS device.
		 */
		uint32_t ss_level = (config->operation & SPI_CS_ACTIVE_HIGH)
			? SPI_SS_ACTIVE_HIGH : SPI_SS_ACTIVE_LOW;
		SPI_EnableAutoSS(dev_cfg->spi, SPI_SS, ss_level);
		 SPI_DisableAutoSS(dev_cfg->spi);
		LOG_WRN("Disable");
	}

	/* Be able to set TX/RX FIFO threshold, for ex: SPI_SetFIFO(dev_cfg->spi, 2, 2) */

#ifdef CONFIG_SPI_NUMAKER_DMA
	data->dma_cfg_cached = false;
#endif
	data->ctx.config = config;

	return 0;
}

/* Remain TX/RX Data in spi_context TX/RX buffer */
static bool spi_numaker_remain_words(struct spi_numaker_data *data)
{
	return spi_context_tx_on(&data->ctx) || spi_context_rx_on(&data->ctx);
}

/*
 * M484 SPI FIFO depth depends on configured data width:
 *   8-16 bits  (spi_dfs 1-2) -> 8-level FIFO
 *   17-32 bits (spi_dfs 3-4) -> 4-level FIFO
 */
static inline size_t spi_numaker_fifo_depth(uint8_t spi_dfs)
{
	return (spi_dfs <= 2) ? 8 : 4;
}

#define SPI_NUMAKER_RXTH_MAX   7  /* RXTH field is 3 bits [26:24] */

static int spi_numaker_txrx(const struct device *dev, uint8_t spi_dfs)
{
	struct spi_numaker_data *data = dev->data;
	const struct spi_numaker_config *dev_cfg = dev->config;
	struct spi_context *ctx = &data->ctx;
	uint32_t tx_frame, rx_frame;
	uint32_t time_out_cnt;

	LOG_DBG("%s", __func__);

	const size_t fifo_depth = spi_numaker_fifo_depth(spi_dfs);

	while (spi_numaker_remain_words(data)) {
		size_t tx_frames = spi_context_tx_on(ctx) ? (ctx->tx_len / spi_dfs) : 0;
		size_t rx_frames = spi_context_rx_on(ctx) ? (ctx->rx_len / spi_dfs) : 0;
		size_t batch = MIN(MAX(tx_frames, rx_frames), fifo_depth);

		if (batch == 0U) {
			break;
		}

		/* Fill TX FIFO */
		for (size_t i = 0; i < batch; i++) {
			if (spi_context_tx_on(ctx)) {
				tx_frame = (ctx->tx_buf == NULL) ? SPI_NUMAKER_TX_NOP
					: UNALIGNED_GET((uint32_t *)ctx->tx_buf);
				SPI_WRITE_TX(dev_cfg->spi, tx_frame);
				spi_context_update_tx(ctx, spi_dfs, 1);
			} else {
				SPI_WRITE_TX(dev_cfg->spi, SPI_NUMAKER_TX_NOP);
			}
		}

		/* Wait for all words to be clocked out */
		time_out_cnt = SystemCoreClock;
		while (SPI_IS_BUSY(dev_cfg->spi)) {
			if (--time_out_cnt == 0) {
				LOG_ERR("Wait for SPI time-out");
				return -EIO;
			}
		}

		/* Drain RX FIFO */
		for (size_t i = 0; i < batch; i++) {
			rx_frame = SPI_READ_RX(dev_cfg->spi);
			if (spi_context_rx_on(ctx)) {
				if (ctx->rx_buf != NULL) {
					if (spi_dfs > 2) {
						UNALIGNED_PUT(rx_frame,
						(uint32_t *)ctx->rx_buf);
					} else if (spi_dfs > 1) {
						UNALIGNED_PUT(rx_frame,
						(uint16_t *)ctx->rx_buf);
					} else {
						UNALIGNED_PUT(rx_frame,
						(uint8_t *)ctx->rx_buf);
					}
				}
				spi_context_update_rx(ctx, spi_dfs, 1);
			}
		}
	}

	LOG_DBG("%s --> exit", __func__);
	return 0;
}

#ifdef CONFIG_SPI_NUMAKER_DMA
static bool spi_numaker_use_dma(const struct device *dev, uint8_t spi_dfs)
{
	const struct spi_numaker_config *dev_cfg = dev->config;

	/* 24-bit frame width is not natively handled in PDMA path. */
	return dev_cfg->dma_enabled && (spi_dfs != 3U);
}

/* Forward declarations for async DMA */
static int spi_numaker_dma_config(const struct device *dev, uint32_t channel, uint32_t slot,
				  enum dma_channel_direction direction, uint32_t src_addr,
				  uint32_t dst_addr, uint32_t size, uint8_t spi_dfs,
				  uint32_t src_adj, uint32_t dst_adj);
#ifdef CONFIG_SPI_ASYNC
static int spi_numaker_dma_start_chunk(const struct device *dev);
static void spi_numaker_async_dma_finish_chunk(const struct device *dev);
#endif

static void spi_numaker_dma_cb(const struct device *dma_dev, void *arg, uint32_t channel, int status)
{
	const struct device *dev = arg;
	const struct spi_numaker_config *dev_cfg = dev->config;
	struct spi_numaker_data *data = dev->data;

	ARG_UNUSED(dma_dev);

	if (status < 0) {
		LOG_ERR("DMA callback error status=%d channel=%u", status, channel);
		data->dma_status = status;
#ifdef CONFIG_SPI_ASYNC
		if (data->async_dma) {
			spi_numaker_async_dma_finish_chunk(dev);
			return;
		}
#endif
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

#ifdef CONFIG_SPI_ASYNC
	if (data->async_dma) {
		/* Wait until both TX and RX channels are done */
		if (data->dma_done == (SPI_NUMAKER_DMA_TX_DONE | SPI_NUMAKER_DMA_RX_DONE)) {
			spi_numaker_async_dma_finish_chunk(dev);
		}
		return;
	}
#endif
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

		uint32_t tx_src_adj = spi_context_tx_buf_on(ctx) ?
			DMA_ADDR_ADJ_INCREMENT : DMA_ADDR_ADJ_NO_CHANGE;
		uint32_t rx_dst_adj = spi_context_rx_buf_on(ctx) ?
			DMA_ADDR_ADJ_INCREMENT : DMA_ADDR_ADJ_NO_CHANGE;

		if (data->dma_cfg_cached &&
		    tx_src_adj == data->dma_tx_src_adj &&
		    rx_dst_adj == data->dma_rx_dst_adj) {
			ret = dma_reload(dev_cfg->dma_dev,
					 dev_cfg->tx_dma_channel, tx_addr,
					 (uint32_t)(uintptr_t)&dev_cfg->spi->TX,
					 chunk_bytes);
			if (ret < 0) {
				return ret;
			}

			ret = dma_reload(dev_cfg->dma_dev,
					 dev_cfg->rx_dma_channel,
					 (uint32_t)(uintptr_t)&dev_cfg->spi->RX,
					 rx_addr, chunk_bytes);
			if (ret < 0) {
				return ret;
			}
		} else {
			ret = spi_numaker_dma_config(dev,
					dev_cfg->tx_dma_channel,
					dev_cfg->tx_dma_slot,
					MEMORY_TO_PERIPHERAL, tx_addr,
					(uint32_t)(uintptr_t)&dev_cfg->spi->TX,
					chunk_bytes, spi_dfs,
					tx_src_adj,
					DMA_ADDR_ADJ_NO_CHANGE);
			if (ret < 0) {
				return ret;
			}

			ret = spi_numaker_dma_config(dev,
					dev_cfg->rx_dma_channel,
					dev_cfg->rx_dma_slot,
					PERIPHERAL_TO_MEMORY,
					(uint32_t)(uintptr_t)&dev_cfg->spi->RX,
					rx_addr, chunk_bytes, spi_dfs,
					DMA_ADDR_ADJ_NO_CHANGE,
					rx_dst_adj);
			if (ret < 0) {
				return ret;
			}

			data->dma_cfg_cached = true;
			data->dma_tx_src_adj = tx_src_adj;
			data->dma_rx_dst_adj = rx_dst_adj;
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

#ifdef CONFIG_SPI_ASYNC
/*
 * Launch one DMA chunk for the current spi_context buffer pointers.
 * Returns 0 on success (DMA started), negative errno on failure.
 * Called from thread context (first chunk) or DMA ISR (subsequent chunks).
 */
static int spi_numaker_dma_start_chunk(const struct device *dev)
{
	struct spi_numaker_data *data = dev->data;
	const struct spi_numaker_config *dev_cfg = dev->config;
	struct spi_context *ctx = &data->ctx;
	uint8_t spi_dfs = data->spi_dfs;
	int ret;

	if (!spi_numaker_remain_words(data)) {
		return 1; /* no more chunks */
	}

	size_t chunk_frames = spi_context_max_continuous_chunk(ctx);

	if (chunk_frames == 0U) {
		return -EINVAL;
	}

	size_t chunk_bytes = chunk_frames * spi_dfs;
	uint32_t tx_addr = spi_context_tx_buf_on(ctx) ? (uint32_t)(uintptr_t)ctx->tx_buf :
		(uint32_t)(uintptr_t)&data->dma_dummy_tx;
	uint32_t rx_addr = spi_context_rx_buf_on(ctx) ? (uint32_t)(uintptr_t)ctx->rx_buf :
		(uint32_t)(uintptr_t)&data->dma_dummy_rx;
	uint32_t tx_src_adj = spi_context_tx_buf_on(ctx) ?
		DMA_ADDR_ADJ_INCREMENT : DMA_ADDR_ADJ_NO_CHANGE;
	uint32_t rx_dst_adj = spi_context_rx_buf_on(ctx) ?
		DMA_ADDR_ADJ_INCREMENT : DMA_ADDR_ADJ_NO_CHANGE;

	if (data->dma_cfg_cached &&
	    tx_src_adj == data->dma_tx_src_adj &&
	    rx_dst_adj == data->dma_rx_dst_adj) {
		ret = dma_reload(dev_cfg->dma_dev, dev_cfg->tx_dma_channel,
				 tx_addr, (uint32_t)(uintptr_t)&dev_cfg->spi->TX,
				 chunk_bytes);
		if (ret < 0) {
			return ret;
		}
		ret = dma_reload(dev_cfg->dma_dev, dev_cfg->rx_dma_channel,
				 (uint32_t)(uintptr_t)&dev_cfg->spi->RX,
				 rx_addr, chunk_bytes);
		if (ret < 0) {
			return ret;
		}
	} else {
		ret = spi_numaker_dma_config(dev, dev_cfg->tx_dma_channel,
				dev_cfg->tx_dma_slot, MEMORY_TO_PERIPHERAL,
				tx_addr, (uint32_t)(uintptr_t)&dev_cfg->spi->TX,
				chunk_bytes, spi_dfs, tx_src_adj,
				DMA_ADDR_ADJ_NO_CHANGE);
		if (ret < 0) {
			return ret;
		}

		ret = spi_numaker_dma_config(dev, dev_cfg->rx_dma_channel,
				dev_cfg->rx_dma_slot, PERIPHERAL_TO_MEMORY,
				(uint32_t)(uintptr_t)&dev_cfg->spi->RX,
				rx_addr, chunk_bytes, spi_dfs,
				DMA_ADDR_ADJ_NO_CHANGE, rx_dst_adj);
		if (ret < 0) {
			return ret;
		}

		data->dma_cfg_cached = true;
		data->dma_tx_src_adj = tx_src_adj;
		data->dma_rx_dst_adj = rx_dst_adj;
	}

	data->dma_done = 0U;
	data->dma_status = 0;

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

	return 0;
}

/*
 * Called from the DMA ISR when both TX+RX are done (or on error) during
 * an async transfer.  Advances spi_context buffers and either launches
 * the next chunk or completes the transfer.
 */
static void spi_numaker_async_dma_finish_chunk(const struct device *dev)
{
	struct spi_numaker_data *data = dev->data;
	const struct spi_numaker_config *dev_cfg = dev->config;
	struct spi_context *ctx = &data->ctx;
	int ret;

	SPI_DISABLE_RX_PDMA(dev_cfg->spi);
	SPI_DISABLE_TX_PDMA(dev_cfg->spi);
	dev_cfg->spi->PDMACTL |= SPI_PDMACTL_PDMARST_Msk;
	(void)dma_stop(dev_cfg->dma_dev, dev_cfg->tx_dma_channel);
	(void)dma_stop(dev_cfg->dma_dev, dev_cfg->rx_dma_channel);

	if (data->dma_status < 0) {
		SPI_ClearRxFIFO(dev_cfg->spi);
		SPI_ClearTxFIFO(dev_cfg->spi);
		goto complete;
	}

	/* Advance spi_context buffer pointers past this chunk */
	size_t chunk_frames = spi_context_max_continuous_chunk(ctx);

	spi_context_update_tx(ctx, data->spi_dfs, chunk_frames);
	spi_context_update_rx(ctx, data->spi_dfs, chunk_frames);

	/* Start next chunk if there is more data */
	ret = spi_numaker_dma_start_chunk(dev);
	if (ret == 0) {
		return; /* next chunk started, wait for DMA callback */
	}

	if (ret < 0) {
		data->dma_status = ret;
	}

	SPI_ClearRxFIFO(dev_cfg->spi);
	SPI_ClearTxFIFO(dev_cfg->spi);

complete:
	data->async_dma = false;

	if (!(ctx->config->operation & SPI_LOCK_ON)) {
		spi_context_cs_control(ctx, false);
		SPI_DISABLE(dev_cfg->spi);
	}

	spi_context_complete(ctx, dev, data->dma_status);
}
#endif /* CONFIG_SPI_ASYNC */
#endif /* CONFIG_SPI_NUMAKER_DMA */

#ifdef CONFIG_SPI_ASYNC
static void spi_numaker_async_work_handler(struct k_work *work)
{
	struct spi_numaker_data *data = CONTAINER_OF(work, struct spi_numaker_data, async_work);
	const struct device *dev = data->dev;
	const struct spi_numaker_config *dev_cfg = dev->config;
	struct spi_context *ctx = &data->ctx;
	int ret;

	ret = spi_numaker_txrx(dev, data->spi_dfs);

	spi_context_cs_control(ctx, false);

	if (!(ctx->config->operation & SPI_LOCK_ON)) {
		SPI_DISABLE(dev_cfg->spi);
	}

	spi_context_complete(ctx, dev, ret);
}

static void spi_numaker_isr(const struct device *dev)
{
	struct spi_numaker_data *data = dev->data;
	const struct spi_numaker_config *dev_cfg = dev->config;
	struct spi_context *ctx = &data->ctx;
	SPI_T *spi = dev_cfg->spi;
	uint8_t spi_dfs = data->spi_dfs;
	uint32_t rx_frame;

	/* Disable RX threshold interrupt while processing */
	SPI_DisableInt(spi, SPI_FIFO_RXTH_INT_MASK);

	/* Ensure last byte is fully shifted in */
	while (SPI_IS_BUSY(spi)) {
		;
	}

	/* Drain RX FIFO for the batch we just sent */
	for (size_t i = 0; i < data->async_irq_batch; i++) {
		rx_frame = SPI_READ_RX(spi);
		if (spi_context_rx_on(ctx)) {
			if (ctx->rx_buf != NULL) {
				if (spi_dfs > 2) {
					UNALIGNED_PUT(rx_frame, (uint32_t *)ctx->rx_buf);
				} else if (spi_dfs > 1) {
					UNALIGNED_PUT(rx_frame, (uint16_t *)ctx->rx_buf);
				} else {
					UNALIGNED_PUT(rx_frame, (uint8_t *)ctx->rx_buf);
				}
			}
			spi_context_update_rx(ctx, spi_dfs, 1);
		}
	}

	/* Check if more data to transfer */
	if (spi_numaker_remain_words(data)) {
		const size_t fifo_depth = spi_numaker_fifo_depth(spi_dfs);
		size_t tx_frames = spi_context_tx_on(ctx) ? (ctx->tx_len / spi_dfs) : 0;
		size_t rx_frames = spi_context_rx_on(ctx) ? (ctx->rx_len / spi_dfs) : 0;
		size_t batch = MIN(MAX(tx_frames, rx_frames), fifo_depth);

		if (batch > 0) {
			/* Fill TX FIFO with next batch */
			for (size_t i = 0; i < batch; i++) {
				if (spi_context_tx_on(ctx)) {
					uint32_t tx_frame = (ctx->tx_buf == NULL)
						? SPI_NUMAKER_TX_NOP
						: UNALIGNED_GET((uint32_t *)ctx->tx_buf);
					SPI_WRITE_TX(spi, tx_frame);
					spi_context_update_tx(ctx, spi_dfs, 1);
				} else {
					SPI_WRITE_TX(spi, SPI_NUMAKER_TX_NOP);
				}
			}

			data->async_irq_batch = batch;
			/* Set RX threshold: fires when RX count > threshold */
			SPI_SetFIFO(spi, 0, MIN(batch - 1, SPI_NUMAKER_RXTH_MAX));
			SPI_EnableInt(spi, SPI_FIFO_RXTH_INT_MASK);
			return;
		}
	}

	/* Transfer complete — only deassert CS if bus will be released.
	 * With SPI_HOLD_ON_CS, spi_context_cs_control(false) is already a
	 * no-op so CS stays asserted.  Only disable the SPI peripheral when
	 * neither LOCK_ON nor HOLD_ON_CS is active — otherwise the next
	 * transfer needs the peripheral running and CS held. */
	if (!(ctx->config->operation & SPI_LOCK_ON)) {
		spi_context_cs_control(ctx, false);
		if (!(ctx->config->operation & SPI_HOLD_ON_CS)) {
			SPI_DISABLE(spi);
		}
	}

	spi_context_complete(ctx, dev, 0);
}
#endif /* CONFIG_SPI_ASYNC */

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

	/* Check if bus is already locked by this config (SPI_LOCK_ON fast path).
	 * When true, CS is already asserted from the first transaction after
	 * lock acquisition — skip CS assert/deassert to avoid gpio overhead. */
	bool cs_held = (config->operation & SPI_LOCK_ON) &&
		       (k_sem_count_get(&ctx->lock) == 0) &&
		       (ctx->owner == config);

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

	/* Cache spi_dfs so the async fast-reentry path can use it when the
	 * bus was locked by a sync transceive (e.g. pre-lock dummy read). */
	data->spi_dfs = spi_dfs;

	SPI_ENABLE(dev_cfg->spi);

	spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, spi_dfs);

	if (!cs_held) {
		spi_context_cs_control(&data->ctx, true);
	}

	/* transceive tx/rx data */
	#ifdef CONFIG_SPI_NUMAKER_DMA
	if (spi_numaker_use_dma(dev, spi_dfs)) {
		LOG_DBG("Using DMA path");
		ret = spi_numaker_txrx_dma(dev, spi_dfs);
	} else
	#endif
	{
		LOG_DBG("Using polling path");
		ret = spi_numaker_txrx(dev, spi_dfs);
	}

	if (!cs_held) {
		spi_context_cs_control(&data->ctx, false);
	}

	/* Keep SPI enabled when bus is locked to avoid enable/disable overhead */
	if (!(ctx->config->operation & SPI_LOCK_ON)) {
		SPI_DISABLE(dev_cfg->spi);
	}

done:
	spi_context_release(ctx, ret);
	LOG_DBG("%s --> [%d]", __func__, ret);
	return ret;
}

static int spi_numaker_release(const struct device *dev, const struct spi_config *config)
{
	struct spi_numaker_data *data = dev->data;
	const struct spi_numaker_config *dev_cfg = dev->config;
	struct spi_context *ctx = &data->ctx;

	if (!spi_context_configured(ctx, config)) {
		return -EINVAL;
	}

	SPI_DISABLE(dev_cfg->spi);
	spi_context_unlock_unconditionally(ctx);

	return 0;
}

#ifdef CONFIG_SPI_ASYNC
static int spi_numaker_transceive_async(const struct device *dev,
					const struct spi_config *config,
					const struct spi_buf_set *tx_bufs,
					const struct spi_buf_set *rx_bufs,
					spi_callback_t cb,
					void *userdata)
{
	struct spi_numaker_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;
	const struct spi_numaker_config *dev_cfg = dev->config;
	int ret;
	uint8_t spi_dfs;

	/* Check if bus is already locked by this config (SPI_LOCK_ON fast path).
	 * When true, SPI is already enabled & configured, CS is asserted, and
	 * spi_dfs is cached — skip configure, enable, CS, and FIFO clears. */
	bool cs_held = (config->operation & SPI_LOCK_ON) &&
		       (k_sem_count_get(&ctx->lock) == 0) &&
		       (ctx->owner == config);

	if (cs_held) {
		/* ── Fast re-entry (ISR hot path) ──
		 * SPI is already enabled & configured, CS is asserted,
		 * spi_dfs is cached.  Skip lock, configure, enable, CS,
		 * FIFO clears, and the generic buffer-setup helpers to
		 * minimise latency from DRDY to first SPI clock.
		 */
		ctx->asynchronous = true;
		ctx->callback = cb;
		ctx->callback_data = userdata;

		spi_dfs = data->spi_dfs;

		/* Inline buffer reset — avoids spi_context_buffers_setup()
		 * and its two spi_context_get_next_buf() calls which do
		 * pointer chasing through the buf_set structs. */
		ctx->current_tx = tx_bufs ? tx_bufs->buffers : NULL;
		ctx->tx_count = tx_bufs ? tx_bufs->count : 0;
		ctx->tx_buf = NULL;
		ctx->tx_len = 0;
		if (tx_bufs && tx_bufs->count && tx_bufs->buffers[0].len) {
			ctx->tx_buf = (const uint8_t *)tx_bufs->buffers[0].buf;
			ctx->tx_len = tx_bufs->buffers[0].len;
		}

		ctx->current_rx = rx_bufs ? rx_bufs->buffers : NULL;
		ctx->rx_count = rx_bufs ? rx_bufs->count : 0;
		ctx->rx_buf = NULL;
		ctx->rx_len = 0;
		if (rx_bufs && rx_bufs->count && rx_bufs->buffers[0].len) {
			ctx->rx_buf = (uint8_t *)rx_bufs->buffers[0].buf;
			ctx->rx_len = rx_bufs->buffers[0].len;
		}

		/* Compute batch and fill TX FIFO directly */
		{
			size_t frames = MAX(ctx->tx_len, ctx->rx_len);
			size_t batch = MIN(frames, spi_numaker_fifo_depth(spi_dfs));

			if (batch == 0) {
				spi_context_complete(ctx, dev, 0);
				return 0;
			}

			/* Fill TX FIFO — for RX-only transfers (common ADC
			 * case) tx_len is 0 so we just write NOPs. */
			if (ctx->tx_len) {
				for (size_t i = 0; i < batch; i++) {
					uint32_t f = (ctx->tx_buf == NULL)
						? SPI_NUMAKER_TX_NOP
						: UNALIGNED_GET((uint32_t *)ctx->tx_buf);
					SPI_WRITE_TX(dev_cfg->spi, f);
					spi_context_update_tx(ctx, spi_dfs, 1);
				}
			} else {
				for (size_t i = 0; i < batch; i++) {
					SPI_WRITE_TX(dev_cfg->spi, SPI_NUMAKER_TX_NOP);
				}
			}

			data->async_irq_batch = batch;
			SPI_SetFIFO(dev_cfg->spi, 0,
				    MIN(batch - 1, SPI_NUMAKER_RXTH_MAX));
			SPI_EnableInt(dev_cfg->spi, SPI_FIFO_RXTH_INT_MASK);
		}
		return 0;
	}

	LOG_DBG("%s", __func__);

	spi_context_lock(ctx, true, cb, userdata, config);

	ret = spi_numaker_configure(dev, config);
	if (ret < 0) {
		goto done;
	}

	{
		uint8_t word_size = SPI_WORD_SIZE_GET(ctx->config->operation);

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
			LOG_ERR("Async does not support %d-bit word size", word_size);
			ret = -ENOTSUP;
			goto done;
		}
	}

#ifdef CONFIG_SPI_NUMAKER_DMA
	if (spi_numaker_use_dma(dev, spi_dfs)) {
		/* DMA async path */
		SPI_ENABLE(dev_cfg->spi);

		spi_context_buffers_setup(ctx, tx_bufs, rx_bufs, spi_dfs);

		data->spi_dfs = spi_dfs;
		data->async_dma = true;

		spi_context_cs_control(ctx, true);

		SPI_DISABLE_RX_PDMA(dev_cfg->spi);
		SPI_DISABLE_TX_PDMA(dev_cfg->spi);
		dev_cfg->spi->PDMACTL |= SPI_PDMACTL_PDMARST_Msk;
		SPI_ClearRxFIFO(dev_cfg->spi);
		SPI_ClearTxFIFO(dev_cfg->spi);

		ret = spi_numaker_dma_start_chunk(dev);
		if (ret != 0) {
			data->async_dma = false;
			spi_context_cs_control(ctx, false);
			if (!(ctx->config->operation & SPI_LOCK_ON)) {
				SPI_DISABLE(dev_cfg->spi);
			}
			if (ret > 0) {
				ret = 0;
				spi_context_complete(ctx, dev, 0);
			}
			goto done;
		}

		return 0;
	}
#endif /* CONFIG_SPI_NUMAKER_DMA */

	/* IRQ-based async path: fill FIFO and let ISR handle the rest */
	SPI_ENABLE(dev_cfg->spi);

	spi_context_buffers_setup(ctx, tx_bufs, rx_bufs, spi_dfs);

	data->spi_dfs = spi_dfs;

	spi_context_cs_control(ctx, true);

	SPI_ClearRxFIFO(dev_cfg->spi);
	SPI_ClearTxFIFO(dev_cfg->spi);

	/* First-time async path (not fast re-entry) */
	{
		const size_t fifo_depth = spi_numaker_fifo_depth(spi_dfs);
		size_t tx_frames = spi_context_tx_on(ctx) ? (ctx->tx_len / spi_dfs) : 0;
		size_t rx_frames = spi_context_rx_on(ctx) ? (ctx->rx_len / spi_dfs) : 0;
		size_t batch = MIN(MAX(tx_frames, rx_frames), fifo_depth);

		if (batch == 0) {
			spi_context_cs_control(ctx, false);
			if (!(ctx->config->operation & SPI_LOCK_ON)) {
				SPI_DISABLE(dev_cfg->spi);
			}
			spi_context_complete(ctx, dev, 0);
			return 0;
		}

		/* Fill TX FIFO with first batch */
		for (size_t i = 0; i < batch; i++) {
			if (spi_context_tx_on(ctx)) {
				uint32_t tx_frame = (ctx->tx_buf == NULL)
					? SPI_NUMAKER_TX_NOP
					: UNALIGNED_GET((uint32_t *)ctx->tx_buf);
				SPI_WRITE_TX(dev_cfg->spi, tx_frame);
				spi_context_update_tx(ctx, spi_dfs, 1);
			} else {
				SPI_WRITE_TX(dev_cfg->spi, SPI_NUMAKER_TX_NOP);
			}
		}

		data->async_irq_batch = batch;
		/* Set RX threshold: interrupt fires when RX count > threshold */
		SPI_SetFIFO(dev_cfg->spi, 0, MIN(batch - 1, SPI_NUMAKER_RXTH_MAX));
		SPI_EnableInt(dev_cfg->spi, SPI_FIFO_RXTH_INT_MASK);
	}
	return 0;

done:
	spi_context_release(ctx, ret);
	return ret;
}
#endif /* CONFIG_SPI_ASYNC */

static DEVICE_API(spi, spi_numaker_driver_api) = {
	.transceive = spi_numaker_transceive,
#ifdef CONFIG_SPI_ASYNC
	.transceive_async = spi_numaker_transceive_async,
#endif
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

#ifdef CONFIG_SPI_ASYNC
	data->dev = dev;
	k_work_init(&data->async_work, spi_numaker_async_work_handler);
	if (dev_cfg->irq_config_func) {
		dev_cfg->irq_config_func(dev);
	}
#endif

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

#ifdef CONFIG_SPI_ASYNC
#define NUMAKER_SPI_IRQ_FUNC_DEFINE(inst)                                                          \
	static void spi_numaker_irq_config_##inst(const struct device *dev)                         \
	{                                                                                           \
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority),                        \
			    spi_numaker_isr, DEVICE_DT_INST_GET(inst), 0);                          \
		irq_enable(DT_INST_IRQN(inst));                                                     \
	}
#define NUMAKER_SPI_IRQ_CONFIG(inst) .irq_config_func = spi_numaker_irq_config_##inst,
#else
#define NUMAKER_SPI_IRQ_FUNC_DEFINE(inst)
#define NUMAKER_SPI_IRQ_CONFIG(inst)
#endif

#define NUMAKER_SPI_INIT(inst)                                                                     \
	PINCTRL_DT_INST_DEFINE(inst);                                                              \
	NUMAKER_SPI_IRQ_FUNC_DEFINE(inst)                                                          \
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
		NUMAKER_SPI_IRQ_CONFIG(inst)                                                       \
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
