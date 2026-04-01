/*
 * SPDX-License-Identifier: Apache-2.0
 *
 *  Copyright (c) 2023 Nuvoton Technology Corporation.
 */

#define DT_DRV_COMPAT nuvoton_numaker_uart

#include <string.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/clock_control_numaker.h>
#include <zephyr/drivers/pinctrl.h>
#ifdef CONFIG_UART_ASYNC_API
#include <zephyr/drivers/dma.h>
#endif
#include <NuMicro.h>

LOG_MODULE_REGISTER(numaker_uart, LOG_LEVEL_ERR);

struct uart_numaker_config {
	UART_T *uart;
	const struct reset_dt_spec reset;
	uint32_t clk_modidx;
	uint32_t clk_src;
	uint32_t clk_div;
	const struct device *clk_dev;
	uint32_t irq_n;
#if defined(CONFIG_UART_INTERRUPT_DRIVEN) || defined(CONFIG_UART_ASYNC_API)
	void (*irq_config_func)(const struct device *dev);
#endif
	const struct pinctrl_dev_config *pincfg;
#ifdef CONFIG_UART_ASYNC_API
	const struct device *dma_dev;
	uint32_t tx_dma_channel;
	uint32_t tx_dma_slot;
	uint32_t rx_dma_channel;
	uint32_t rx_dma_slot;
#endif
};

struct uart_numaker_data {
	const struct device *clock;
	struct uart_config ucfg;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t user_cb;
	void *user_data;
#endif
#ifdef CONFIG_UART_ASYNC_API
	const struct device *dev;
	uart_callback_t async_cb;
	void *async_cb_data;
	/* TX state */
	const uint8_t *tx_buf;
	size_t tx_buf_len;
	struct dma_block_config tx_blk;
	/* RX state */
	uint8_t *rx_buf;
	size_t rx_buf_len;
	size_t rx_buf_offset;
	uint8_t *rx_next_buf;
	size_t rx_next_buf_len;
	struct dma_block_config rx_blk;
	int32_t rx_timeout_us;
	struct k_work_delayable rx_timeout_work;
	bool rx_enabled;
#endif
};

static int uart_numaker_poll_in(const struct device *dev, unsigned char *c)
{
	const struct uart_numaker_config *config = dev->config;
	uint32_t count;

	count = UART_Read(config->uart, c, 1);
	if (!count) {
		return -1;
	}

	return 0;
}

static void uart_numaker_poll_out(const struct device *dev, unsigned char c)
{
	const struct uart_numaker_config *config = dev->config;

	UART_Write(config->uart, &c, 1);
}

static int uart_numaker_err_check(const struct device *dev)
{
	const struct uart_numaker_config *config = dev->config;
	UART_T *uart = config->uart;
	uint32_t flags = uart->FIFOSTS;
	int err = 0;

	if (flags & UART_FIFOSTS_RXOVIF_Msk) {
		err |= UART_ERROR_OVERRUN;
	}

	if (flags & UART_FIFOSTS_PEF_Msk) {
		err |= UART_ERROR_PARITY;
	}

	if (flags & UART_FIFOSTS_FEF_Msk) {
		err |= UART_ERROR_FRAMING;
	}

	if (flags & UART_FIFOSTS_BIF_Msk) {
		err |= UART_BREAK;
	}

	if (flags & (UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk |
		     UART_FIFOSTS_RXOVIF_Msk)) {
		uart->FIFOSTS = (UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk |
				 UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_RXOVIF_Msk);
	}
	return err;
}

static inline int32_t uart_numaker_convert_stopbit(enum uart_config_stop_bits sb)
{
	switch (sb) {
	case UART_CFG_STOP_BITS_1:
		return UART_STOP_BIT_1;
	case UART_CFG_STOP_BITS_1_5:
		return UART_STOP_BIT_1_5;
	case UART_CFG_STOP_BITS_2:
		return UART_STOP_BIT_2;
	default:
		return -ENOTSUP;
	}
};

static inline int32_t uart_numaker_convert_datalen(enum uart_config_data_bits db)
{
	switch (db) {
	case UART_CFG_DATA_BITS_5:
		return UART_WORD_LEN_5;
	case UART_CFG_DATA_BITS_6:
		return UART_WORD_LEN_6;
	case UART_CFG_DATA_BITS_7:
		return UART_WORD_LEN_7;
	case UART_CFG_DATA_BITS_8:
		return UART_WORD_LEN_8;
	default:
		return -ENOTSUP;
	}
}

static inline uint32_t uart_numaker_convert_parity(enum uart_config_parity parity)
{
	switch (parity) {
	case UART_CFG_PARITY_ODD:
		return UART_PARITY_ODD;
	case UART_CFG_PARITY_EVEN:
		return UART_PARITY_EVEN;
	case UART_CFG_PARITY_MARK:
		return UART_PARITY_MARK;
	case UART_CFG_PARITY_SPACE:
		return UART_PARITY_SPACE;
	case UART_CFG_PARITY_NONE:
	default:
		return UART_PARITY_NONE;
	}
}

#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
static int uart_numaker_configure(const struct device *dev, const struct uart_config *cfg)
{
	const struct uart_numaker_config *config = dev->config;
	struct uart_numaker_data *pData = dev->data;
	int32_t databits, stopbits;
	uint32_t parity;

	databits = uart_numaker_convert_datalen(cfg->data_bits);
	if (databits < 0) {
		return databits;
	}

	stopbits = uart_numaker_convert_stopbit(cfg->stop_bits);
	if (stopbits < 0) {
		return stopbits;
	}

	if (cfg->flow_ctrl == UART_CFG_FLOW_CTRL_NONE) {
		UART_DisableFlowCtrl(config->uart);
	} else if (cfg->flow_ctrl == UART_CFG_FLOW_CTRL_RTS_CTS) {
		UART_EnableFlowCtrl(config->uart);
	} else {
		return -ENOTSUP;
	}

	parity = uart_numaker_convert_parity(cfg->parity);

	UART_SetLineConfig(config->uart, cfg->baudrate, databits, parity, stopbits);

	memcpy(&pData->ucfg, cfg, sizeof(*cfg));

	return 0;
}

static int uart_numaker_config_get(const struct device *dev, struct uart_config *cfg)
{
	struct uart_numaker_data *pData = dev->data;

	memcpy(cfg, &pData->ucfg, sizeof(*cfg));

	return 0;
}
#endif /* CONFIG_UART_USE_RUNTIME_CONFIGURE */

#ifdef CONFIG_UART_ASYNC_API

static void async_user_callback(const struct device *dev, struct uart_event *evt)
{
	struct uart_numaker_data *data = dev->data;

	if (data->async_cb) {
		data->async_cb(dev, evt, data->async_cb_data);
	}
}

static void async_evt_tx_done(const struct device *dev)
{
	struct uart_numaker_data *data = dev->data;
	struct uart_event evt = {
		.type = UART_TX_DONE,
		.data.tx.buf = data->tx_buf,
		.data.tx.len = data->tx_buf_len,
	};

	data->tx_buf = NULL;
	data->tx_buf_len = 0;
	async_user_callback(dev, &evt);
}

static void async_evt_tx_aborted(const struct device *dev)
{
	struct uart_numaker_data *data = dev->data;
	const struct uart_numaker_config *config = dev->config;
	struct dma_status stat;
	size_t bytes_sent = data->tx_buf_len;

	if (dma_get_status(config->dma_dev, config->tx_dma_channel, &stat) == 0) {
		bytes_sent = data->tx_buf_len - stat.pending_length;
	}

	struct uart_event evt = {
		.type = UART_TX_ABORTED,
		.data.tx.buf = data->tx_buf,
		.data.tx.len = bytes_sent,
	};

	data->tx_buf = NULL;
	data->tx_buf_len = 0;
	async_user_callback(dev, &evt);
}

static void async_evt_rx_rdy(const struct device *dev)
{
	struct uart_numaker_data *data = dev->data;
	const struct uart_numaker_config *config = dev->config;
	struct dma_status stat;
	size_t total_received;

	if (dma_get_status(config->dma_dev, config->rx_dma_channel, &stat) == 0) {
		total_received = data->rx_buf_len - stat.pending_length;
	} else {
		total_received = data->rx_buf_len;
	}

	size_t new_bytes = total_received - data->rx_buf_offset;

	if (new_bytes == 0) {
		return;
	}

	struct uart_event evt = {
		.type = UART_RX_RDY,
		.data.rx.buf = data->rx_buf,
		.data.rx.offset = data->rx_buf_offset,
		.data.rx.len = new_bytes,
	};

	data->rx_buf_offset = total_received;
	async_user_callback(dev, &evt);
}

static void async_evt_rx_buf_request(const struct device *dev)
{
	struct uart_event evt = {
		.type = UART_RX_BUF_REQUEST,
	};

	async_user_callback(dev, &evt);
}

static void async_evt_rx_buf_release(const struct device *dev, uint8_t *buf)
{
	struct uart_event evt = {
		.type = UART_RX_BUF_RELEASED,
		.data.rx_buf.buf = buf,
	};

	async_user_callback(dev, &evt);
}

static void async_evt_rx_disabled(const struct device *dev)
{
	struct uart_event evt = {
		.type = UART_RX_DISABLED,
	};

	async_user_callback(dev, &evt);
}

static void uart_numaker_dma_tx_cb(const struct device *dma_dev, void *arg,
				    uint32_t channel, int status)
{
	const struct device *dev = arg;
	const struct uart_numaker_config *config = dev->config;

	ARG_UNUSED(dma_dev);
	ARG_UNUSED(channel);

	/* Disable UART TX PDMA */
	UART_PDMA_DISABLE(config->uart, UART_INTEN_TXPDMAEN_Msk);

	if (status != 0) {
		LOG_ERR("TX DMA error: %d", status);
	}

	async_evt_tx_done(dev);
}

static void uart_numaker_rx_timeout_work(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct uart_numaker_data *data =
		CONTAINER_OF(dwork, struct uart_numaker_data, rx_timeout_work);
	const struct device *dev = data->dev;

	if (!data->rx_enabled || data->rx_buf == NULL) {
		return;
	}

	async_evt_rx_rdy(dev);

	/* Reschedule if still receiving */
	if (data->rx_enabled && data->rx_timeout_us != SYS_FOREVER_US) {
		k_work_reschedule(&data->rx_timeout_work,
				  K_USEC(data->rx_timeout_us));
	}
}

static void uart_numaker_dma_rx_cb(const struct device *dma_dev, void *arg,
				    uint32_t channel, int status)
{
	const struct device *dev = arg;
	struct uart_numaker_data *data = dev->data;
	const struct uart_numaker_config *config = dev->config;

	ARG_UNUSED(dma_dev);
	ARG_UNUSED(channel);

	if (status != 0) {
		LOG_ERR("RX DMA error: %d", status);
	}

	/* Cancel timeout */
	k_work_cancel_delayable(&data->rx_timeout_work);

	/* DMA buffer is full — fire RX_RDY for all bytes not yet reported */
	if (data->rx_buf_len > data->rx_buf_offset) {
		struct uart_event rx_rdy_evt = {
			.type = UART_RX_RDY,
			.data.rx.buf = data->rx_buf,
			.data.rx.offset = data->rx_buf_offset,
			.data.rx.len = data->rx_buf_len - data->rx_buf_offset,
		};
		data->rx_buf_offset = data->rx_buf_len;
		async_user_callback(dev, &rx_rdy_evt);
	}

	/* Release current buffer */
	uint8_t *old_buf = data->rx_buf;

	/* Switch to next buffer if available */
	if (data->rx_next_buf != NULL && data->rx_next_buf_len > 0) {
		data->rx_buf = data->rx_next_buf;
		data->rx_buf_len = data->rx_next_buf_len;
		data->rx_buf_offset = 0;
		data->rx_next_buf = NULL;
		data->rx_next_buf_len = 0;

		/* Release old buffer */
		async_evt_rx_buf_release(dev, old_buf);

		/* Configure DMA for new buffer */
		struct dma_config dma_cfg = {0};
		struct dma_block_config blk = {0};

		dma_cfg.channel_direction = PERIPHERAL_TO_MEMORY;
		dma_cfg.source_data_size = 1;
		dma_cfg.dest_data_size = 1;
		dma_cfg.source_burst_length = 1;
		dma_cfg.dest_burst_length = 1;
		dma_cfg.source_handshake = 1;
		dma_cfg.dma_slot = config->rx_dma_slot;
		dma_cfg.dma_callback = uart_numaker_dma_rx_cb;
		dma_cfg.user_data = (void *)dev;
		dma_cfg.complete_callback_en = 1;
		dma_cfg.block_count = 1;
		dma_cfg.head_block = &blk;

		blk.source_address = (uint32_t)&config->uart->DAT;
		blk.dest_address = (uint32_t)data->rx_buf;
		blk.block_size = data->rx_buf_len;
		blk.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		blk.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;

		dma_config(config->dma_dev, config->rx_dma_channel, &dma_cfg);
		dma_start(config->dma_dev, config->rx_dma_channel);

		/* Request next buffer from user */
		async_evt_rx_buf_request(dev);

		/* Restart timeout */
		if (data->rx_timeout_us != SYS_FOREVER_US) {
			k_work_reschedule(&data->rx_timeout_work,
					  K_USEC(data->rx_timeout_us));
		}
	} else {
		/* No next buffer — disable RX */
		UART_PDMA_DISABLE(config->uart, UART_INTEN_RXPDMAEN_Msk);
		data->rx_enabled = false;
		data->rx_buf = NULL;
		data->rx_buf_len = 0;

		async_evt_rx_buf_release(dev, old_buf);
		async_evt_rx_disabled(dev);
	}
}

static void uart_numaker_async_isr(const struct device *dev)
{
	struct uart_numaker_data *data = dev->data;
	const struct uart_numaker_config *config = dev->config;
	UART_T *uart = config->uart;
	uint32_t intsts = uart->INTSTS;

	/* DMA-mode RX timeout: partial data arrived */
	if (intsts & UART_INTSTS_HWTOINT_Msk) {
		/* Clear the timeout flag by reading RX FIFO — but in DMA mode,
		 * the PDMA should have already transferred bytes. Just flush
		 * the event for whatever has been received so far.
		 */
		if (data->rx_enabled && data->rx_buf != NULL) {
			async_evt_rx_rdy(dev);

			/* Reschedule timeout */
			if (data->rx_timeout_us != SYS_FOREVER_US) {
				k_work_reschedule(&data->rx_timeout_work,
						  K_USEC(data->rx_timeout_us));
			}
		}
	}

	/* DMA-mode RX line status error */
	if (intsts & UART_INTSTS_HWRLSIF_Msk) {
		/* Clear errors */
		uart->FIFOSTS = (UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk |
				 UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_RXOVIF_Msk);
	}

	/* DMA-mode buffer error */
	if (intsts & UART_INTSTS_HWBUFEIF_Msk) {
		uart->FIFOSTS = (UART_FIFOSTS_RXOVIF_Msk | UART_FIFOSTS_TXOVIF_Msk);
	}
}

static int uart_numaker_callback_set(const struct device *dev,
				     uart_callback_t callback, void *user_data)
{
	struct uart_numaker_data *data = dev->data;

	data->async_cb = callback;
	data->async_cb_data = user_data;
	return 0;
}

static int uart_numaker_tx(const struct device *dev, const uint8_t *buf,
			   size_t len, int32_t timeout)
{
	struct uart_numaker_data *data = dev->data;
	const struct uart_numaker_config *config = dev->config;
	struct dma_config dma_cfg = {0};
	int ret;

	ARG_UNUSED(timeout);

	if (data->tx_buf != NULL) {
		return -EBUSY;
	}

	data->tx_buf = buf;
	data->tx_buf_len = len;

	dma_cfg.channel_direction = MEMORY_TO_PERIPHERAL;
	dma_cfg.source_data_size = 1;
	dma_cfg.dest_data_size = 1;
	dma_cfg.source_burst_length = 1;
	dma_cfg.dest_burst_length = 1;
	dma_cfg.dest_handshake = 1;
	dma_cfg.dma_slot = config->tx_dma_slot;
	dma_cfg.dma_callback = uart_numaker_dma_tx_cb;
	dma_cfg.user_data = (void *)dev;
	dma_cfg.complete_callback_en = 1;
	dma_cfg.block_count = 1;
	dma_cfg.head_block = &data->tx_blk;

	memset(&data->tx_blk, 0, sizeof(data->tx_blk));
	data->tx_blk.source_address = (uint32_t)buf;
	data->tx_blk.dest_address = (uint32_t)&config->uart->DAT;
	data->tx_blk.block_size = len;
	data->tx_blk.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	data->tx_blk.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;

	ret = dma_config(config->dma_dev, config->tx_dma_channel, &dma_cfg);
	if (ret != 0) {
		LOG_ERR("TX DMA config failed: %d", ret);
		data->tx_buf = NULL;
		return ret;
	}

	ret = dma_start(config->dma_dev, config->tx_dma_channel);
	if (ret != 0) {
		LOG_ERR("TX DMA start failed: %d", ret);
		data->tx_buf = NULL;
		return ret;
	}

	/* Enable UART TX PDMA after DMA channel is ready */
	UART_PDMA_ENABLE(config->uart, UART_INTEN_TXPDMAEN_Msk);

	return 0;
}

static int uart_numaker_tx_abort(const struct device *dev)
{
	struct uart_numaker_data *data = dev->data;
	const struct uart_numaker_config *config = dev->config;

	if (data->tx_buf == NULL) {
		return -EFAULT;
	}

	UART_PDMA_DISABLE(config->uart, UART_INTEN_TXPDMAEN_Msk);
	dma_stop(config->dma_dev, config->tx_dma_channel);
	async_evt_tx_aborted(dev);

	return 0;
}

static int uart_numaker_rx_enable(const struct device *dev, uint8_t *buf,
				  size_t len, int32_t timeout)
{
	struct uart_numaker_data *data = dev->data;
	const struct uart_numaker_config *config = dev->config;
	struct dma_config dma_cfg = {0};
	int ret;

	if (data->rx_enabled) {
		return -EBUSY;
	}

	data->rx_buf = buf;
	data->rx_buf_len = len;
	data->rx_buf_offset = 0;
	data->rx_next_buf = NULL;
	data->rx_next_buf_len = 0;
	data->rx_timeout_us = timeout;
	data->rx_enabled = true;

	dma_cfg.channel_direction = PERIPHERAL_TO_MEMORY;
	dma_cfg.source_data_size = 1;
	dma_cfg.dest_data_size = 1;
	dma_cfg.source_burst_length = 1;
	dma_cfg.dest_burst_length = 1;
	dma_cfg.source_handshake = 1;
	dma_cfg.dma_slot = config->rx_dma_slot;
	dma_cfg.dma_callback = uart_numaker_dma_rx_cb;
	dma_cfg.user_data = (void *)dev;
	dma_cfg.complete_callback_en = 1;
	dma_cfg.block_count = 1;
	dma_cfg.head_block = &data->rx_blk;

	memset(&data->rx_blk, 0, sizeof(data->rx_blk));
	data->rx_blk.source_address = (uint32_t)&config->uart->DAT;
	data->rx_blk.dest_address = (uint32_t)buf;
	data->rx_blk.block_size = len;
	data->rx_blk.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	data->rx_blk.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;

	ret = dma_config(config->dma_dev, config->rx_dma_channel, &dma_cfg);
	if (ret != 0) {
		LOG_ERR("RX DMA config failed: %d", ret);
		data->rx_enabled = false;
		return ret;
	}

	ret = dma_start(config->dma_dev, config->rx_dma_channel);
	if (ret != 0) {
		LOG_ERR("RX DMA start failed: %d", ret);
		data->rx_enabled = false;
		return ret;
	}

	/* Enable UART RX timeout interrupt for partial-buffer notifications in DMA mode */
	UART_SetTimeoutCnt(config->uart, 40);
	config->uart->INTEN |= UART_INTEN_RXTOIEN_Msk | UART_INTEN_TOCNTEN_Msk;

	/* Flush stale RX FIFO data just before enabling PDMA to minimize the window */
	config->uart->FIFO |= UART_FIFO_RXRST_Msk;

	/* Enable UART RX PDMA after DMA channel is ready */
	UART_PDMA_ENABLE(config->uart, UART_INTEN_RXPDMAEN_Msk);

	/* Request next buffer for double-buffering */
	async_evt_rx_buf_request(dev);

	/* Start timeout work if enabled */
	if (timeout != SYS_FOREVER_US) {
		k_work_reschedule(&data->rx_timeout_work, K_USEC(timeout));
	}

	return 0;
}

static int uart_numaker_rx_buf_rsp(const struct device *dev, uint8_t *buf,
				   size_t len)
{
	struct uart_numaker_data *data = dev->data;

	data->rx_next_buf = buf;
	data->rx_next_buf_len = len;

	return 0;
}

static int uart_numaker_rx_disable(const struct device *dev)
{
	struct uart_numaker_data *data = dev->data;
	const struct uart_numaker_config *config = dev->config;

	if (!data->rx_enabled) {
		return -EFAULT;
	}

	k_work_cancel_delayable(&data->rx_timeout_work);
	UART_PDMA_DISABLE(config->uart, UART_INTEN_RXPDMAEN_Msk);
	config->uart->INTEN &= ~(UART_INTEN_RXTOIEN_Msk | UART_INTEN_TOCNTEN_Msk);
	dma_stop(config->dma_dev, config->rx_dma_channel);

	data->rx_enabled = false;

	/* Flush any pending data */
	async_evt_rx_rdy(dev);

	/* Release current buffer */
	if (data->rx_buf != NULL) {
		async_evt_rx_buf_release(dev, data->rx_buf);
		data->rx_buf = NULL;
	}

	/* Release next buffer if queued */
	if (data->rx_next_buf != NULL) {
		async_evt_rx_buf_release(dev, data->rx_next_buf);
		data->rx_next_buf = NULL;
	}

	async_evt_rx_disabled(dev);
	return 0;
}

#endif /* CONFIG_UART_ASYNC_API */

static int uart_numaker_init(const struct device *dev)
{
	const struct uart_numaker_config *config = dev->config;
	struct uart_numaker_data *pData = dev->data;
	int err = 0;

	SYS_UnlockReg();

	struct numaker_scc_subsys scc_subsys;

	memset(&scc_subsys, 0x00, sizeof(scc_subsys));
	scc_subsys.subsys_id = NUMAKER_SCC_SUBSYS_ID_PCC;
	scc_subsys.pcc.clk_modidx = config->clk_modidx;
	scc_subsys.pcc.clk_src = config->clk_src;
	scc_subsys.pcc.clk_div = config->clk_div;

	/* Equivalent to CLK_EnableModuleClock(clk_modidx) */
	err = clock_control_on(config->clk_dev, (clock_control_subsys_t)&scc_subsys);
	if (err != 0) {
		goto move_exit;
	}
	/* Equivalent to CLK_SetModuleClock(clk_modidx, clk_src, clk_div) */
	err = clock_control_configure(config->clk_dev, (clock_control_subsys_t)&scc_subsys, NULL);
	if (err != 0) {
		goto move_exit;
	}

	/*
	 * Set pinctrl for UART0 RXD and TXD
	 * Set multi-function pins for UART0 RXD and TXD
	 */
	err = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
	if (err != 0) {
		goto move_exit;
	}

	/* Same as BSP's SYS_ResetModule(id_rst) */
	if (!device_is_ready(config->reset.dev)) {
		LOG_ERR("reset controller not ready");
		return -ENODEV;
	}

	/* Reset UART to default state */
	reset_line_toggle_dt(&config->reset);

	UART_Open(config->uart, pData->ucfg.baudrate);

	if (pData->ucfg.flow_ctrl == UART_CFG_FLOW_CTRL_NONE) {
		UART_DisableFlowCtrl(config->uart);
	} else if (pData->ucfg.flow_ctrl == UART_CFG_FLOW_CTRL_RTS_CTS) {
		UART_EnableFlowCtrl(config->uart);
	} else {
		LOG_ERR("H/W flow control (%d) not support", pData->ucfg.flow_ctrl);
		return -ENOTSUP;
	}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	config->irq_config_func(dev);
#endif

#ifdef CONFIG_UART_ASYNC_API
	pData->dev = dev;
	k_work_init_delayable(&pData->rx_timeout_work, uart_numaker_rx_timeout_work);

	if (!device_is_ready(config->dma_dev)) {
		LOG_ERR("DMA device not ready");
		return -ENODEV;
	}

	config->irq_config_func(dev);
#endif

move_exit:
	SYS_LockReg();
	return err;
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static int uart_numaker_fifo_fill(const struct device *dev, const uint8_t *tx_data, int size)
{
	const struct uart_numaker_config *config = dev->config;
	UART_T *uart = config->uart;
	int tx_bytes = 0;

	/* Check TX FIFO not full, then fill */
	while (((size - tx_bytes) > 0) && (!(uart->FIFOSTS & UART_FIFOSTS_TXFULL_Msk))) {
		/* Fill one byte into TX FIFO */
		uart->DAT = tx_data[tx_bytes++];
	}

	return tx_bytes;
}

static int uart_numaker_fifo_read(const struct device *dev, uint8_t *rx_data, const int size)
{
	const struct uart_numaker_config *config = dev->config;
	UART_T *uart = config->uart;
	int rx_bytes = 0;

	/* Check RX FIFO not empty, then read */
	while (((size - rx_bytes) > 0) && (!(uart->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk))) {
		/* Read one byte from UART RX FIFO */
		rx_data[rx_bytes++] = (uint8_t)uart->DAT;
	}

	return rx_bytes;
}

static void uart_numaker_irq_tx_enable(const struct device *dev)
{
	const struct uart_numaker_config *config = dev->config;
	UART_T *uart = config->uart;

	UART_EnableInt(uart, UART_INTEN_THREIEN_Msk);
}

static void uart_numaker_irq_tx_disable(const struct device *dev)
{
	const struct uart_numaker_config *config = dev->config;
	UART_T *uart = config->uart;

	UART_DisableInt(uart, UART_INTEN_THREIEN_Msk);
}

static int uart_numaker_irq_tx_ready(const struct device *dev)
{
	const struct uart_numaker_config *config = dev->config;
	UART_T *uart = config->uart;

	return ((!UART_IS_TX_FULL(uart)) && (uart->INTEN & UART_INTEN_THREIEN_Msk));
}

static int uart_numaker_irq_tx_complete(const struct device *dev)
{
	const struct uart_numaker_config *config = dev->config;
	UART_T *uart = config->uart;

	return (uart->INTSTS & UART_INTSTS_THREINT_Msk);
}

static void uart_numaker_irq_rx_enable(const struct device *dev)
{
	const struct uart_numaker_config *config = dev->config;
	UART_T *uart = config->uart;

	UART_EnableInt(uart, UART_INTEN_RDAIEN_Msk);
}

static void uart_numaker_irq_rx_disable(const struct device *dev)
{
	const struct uart_numaker_config *config = dev->config;
	UART_T *uart = config->uart;

	UART_DisableInt(uart, UART_INTEN_RDAIEN_Msk);
}

static int uart_numaker_irq_rx_ready(const struct device *dev)
{
	const struct uart_numaker_config *config = dev->config;
	UART_T *uart = config->uart;

	return ((!UART_GET_RX_EMPTY(uart)) && (uart->INTEN & UART_INTEN_RDAIEN_Msk));
}

static void uart_numaker_irq_err_enable(const struct device *dev)
{
	const struct uart_numaker_config *config = dev->config;
	UART_T *uart = config->uart;

	UART_EnableInt(uart, UART_INTEN_BUFERRIEN_Msk | UART_INTEN_SWBEIEN_Msk);
}

static void uart_numaker_irq_err_disable(const struct device *dev)
{
	const struct uart_numaker_config *config = dev->config;
	UART_T *uart = config->uart;

	UART_DisableInt(uart, UART_INTEN_BUFERRIEN_Msk | UART_INTEN_SWBEIEN_Msk);
}

static int uart_numaker_irq_is_pending(const struct device *dev)
{

	return (uart_numaker_irq_tx_ready(dev) || (uart_numaker_irq_rx_ready(dev)));
}

static int uart_numaker_irq_update(const struct device *dev)
{
	ARG_UNUSED(dev);

	/* nothing to be done */
	return 1;
}

static void uart_numaker_irq_callback_set(const struct device *dev,
					  uart_irq_callback_user_data_t cb, void *cb_data)
{
	struct uart_numaker_data *pData = dev->data;

	pData->user_cb = cb;
	pData->user_data = cb_data;
}

static void uart_numaker_isr(const struct device *dev)
{
	struct uart_numaker_data *pData = dev->data;

	if (pData->user_cb) {
		pData->user_cb(dev, pData->user_data);
	}
}

#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

static DEVICE_API(uart, uart_numaker_driver_api) = {
	.poll_in = uart_numaker_poll_in,
	.poll_out = uart_numaker_poll_out,
	.err_check = uart_numaker_err_check,
#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
	.configure = uart_numaker_configure,
	.config_get = uart_numaker_config_get,
#endif
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = uart_numaker_fifo_fill,
	.fifo_read = uart_numaker_fifo_read,
	.irq_tx_enable = uart_numaker_irq_tx_enable,
	.irq_tx_disable = uart_numaker_irq_tx_disable,
	.irq_tx_ready = uart_numaker_irq_tx_ready,
	.irq_tx_complete = uart_numaker_irq_tx_complete,
	.irq_rx_enable = uart_numaker_irq_rx_enable,
	.irq_rx_disable = uart_numaker_irq_rx_disable,
	.irq_rx_ready = uart_numaker_irq_rx_ready,
	.irq_err_enable = uart_numaker_irq_err_enable,
	.irq_err_disable = uart_numaker_irq_err_disable,
	.irq_is_pending = uart_numaker_irq_is_pending,
	.irq_update = uart_numaker_irq_update,
	.irq_callback_set = uart_numaker_irq_callback_set,
#endif
#ifdef CONFIG_UART_ASYNC_API
	.callback_set = uart_numaker_callback_set,
	.tx = uart_numaker_tx,
	.tx_abort = uart_numaker_tx_abort,
	.rx_enable = uart_numaker_rx_enable,
	.rx_buf_rsp = uart_numaker_rx_buf_rsp,
	.rx_disable = uart_numaker_rx_disable,
#endif
};

#define CLOCK_CTRL_INIT(n) .clk_dev = DEVICE_DT_GET(DT_PARENT(DT_INST_CLOCKS_CTLR(n))),

#define PINCTRL_DEFINE(n) PINCTRL_DT_INST_DEFINE(n);
#define PINCTRL_INIT(n)	  .pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),

#if defined(CONFIG_UART_INTERRUPT_DRIVEN)
#define NUMAKER_UART_IRQ_CONFIG_FUNC(n)                                                            \
	static void uart_numaker_irq_config_##n(const struct device *dev)                          \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), uart_numaker_isr,           \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}
#define IRQ_FUNC_INIT(n) .irq_config_func = uart_numaker_irq_config_##n
#elif defined(CONFIG_UART_ASYNC_API)
#define NUMAKER_UART_IRQ_CONFIG_FUNC(n)                                                            \
	static void uart_numaker_irq_config_##n(const struct device *dev)                          \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), uart_numaker_async_isr,     \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}
#define IRQ_FUNC_INIT(n) .irq_config_func = uart_numaker_irq_config_##n
#else
#define NUMAKER_UART_IRQ_CONFIG_FUNC(n)
#define IRQ_FUNC_INIT(n)
#endif

#ifdef CONFIG_UART_ASYNC_API
#define NUMAKER_UART_DMA_INIT(n)                                                                   \
	.dma_dev = COND_CODE_1(DT_INST_DMAS_HAS_NAME(n, tx),                                      \
		(DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(n, tx))), (NULL)),                        \
	.tx_dma_channel = COND_CODE_1(DT_INST_DMAS_HAS_NAME(n, tx),                               \
		(DT_INST_DMAS_CELL_BY_NAME(n, tx, channel)), (0)),                                  \
	.tx_dma_slot = COND_CODE_1(DT_INST_DMAS_HAS_NAME(n, tx),                                  \
		(DT_INST_DMAS_CELL_BY_NAME(n, tx, slot)), (0)),                                     \
	.rx_dma_channel = COND_CODE_1(DT_INST_DMAS_HAS_NAME(n, rx),                               \
		(DT_INST_DMAS_CELL_BY_NAME(n, rx, channel)), (0)),                                  \
	.rx_dma_slot = COND_CODE_1(DT_INST_DMAS_HAS_NAME(n, rx),                                  \
		(DT_INST_DMAS_CELL_BY_NAME(n, rx, slot)), (0)),
#else
#define NUMAKER_UART_DMA_INIT(n)
#endif

#define NUMAKER_UART_INIT(inst)                                                                    \
	PINCTRL_DEFINE(inst)                                                                       \
	NUMAKER_UART_IRQ_CONFIG_FUNC(inst)                                                         \
                                                                                                   \
	static const struct uart_numaker_config uart_numaker_cfg_##inst = {                        \
		.uart = (UART_T *)DT_INST_REG_ADDR(inst),                                          \
		.reset = RESET_DT_SPEC_INST_GET(inst),                                             \
		.clk_modidx = DT_INST_CLOCKS_CELL(inst, clock_module_index),                       \
		.clk_src = DT_INST_CLOCKS_CELL(inst, clock_source),                                \
		.clk_div = DT_INST_CLOCKS_CELL(inst, clock_divider),                               \
		CLOCK_CTRL_INIT(inst).irq_n = DT_INST_IRQN(inst),                                  \
		PINCTRL_INIT(inst) IRQ_FUNC_INIT(inst),                                            \
		NUMAKER_UART_DMA_INIT(inst)                                                        \
	};                                                                                         \
                                                                                                   \
	static struct uart_numaker_data uart_numaker_data_##inst = {                               \
		.ucfg =                                                                            \
			{                                                                          \
				.baudrate = DT_INST_PROP(inst, current_speed),                     \
				.flow_ctrl = COND_CODE_1(DT_INST_PROP_OR(inst, hw_flow_control, 0),\
							 (UART_CFG_FLOW_CTRL_RTS_CTS),             \
							 (UART_CFG_FLOW_CTRL_NONE)),               \
			},                                                                         \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, uart_numaker_init, NULL, &uart_numaker_data_##inst,            \
			      &uart_numaker_cfg_##inst, PRE_KERNEL_1, CONFIG_SERIAL_INIT_PRIORITY, \
			      &uart_numaker_driver_api);

DT_INST_FOREACH_STATUS_OKAY(NUMAKER_UART_INIT)
