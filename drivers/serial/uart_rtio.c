/*
 * Copyright (c) 2024 iWorx Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/rtio/work.h>
#include <zephyr/drivers/uart/rtio.h>
#include <zephyr/sys/mpsc_lockfree.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(uart_rtio, CONFIG_UART_LOG_LEVEL);

const struct rtio_iodev_api uart_iodev_api = {
	.submit = uart_iodev_submit,
};

/**
 * Async callback used during synchronous fallback submit.
 * Signals the semaphore when TX_DONE or RX_RDY/RX_DISABLED arrive.
 */
struct uart_rtio_sync_ctx {
	struct k_sem sem;
	int result;
	size_t rx_received;
	bool for_rx;
};

static void uart_rtio_async_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
	struct uart_rtio_sync_ctx *ctx = user_data;

	switch (evt->type) {
	case UART_TX_DONE:
		if (!ctx->for_rx) {
			ctx->result = 0;
			k_sem_give(&ctx->sem);
		}
		break;
	case UART_TX_ABORTED:
		if (!ctx->for_rx) {
			ctx->result = -ECANCELED;
			k_sem_give(&ctx->sem);
		}
		break;
	case UART_RX_RDY:
		if (ctx->for_rx) {
			ctx->rx_received += evt->data.rx.len;
		}
		break;
	case UART_RX_DISABLED:
		if (ctx->for_rx) {
			ctx->result = 0;
			k_sem_give(&ctx->sem);
		}
		break;
	case UART_RX_BUF_REQUEST:
		/* Single-buffer operation, no second buffer */
		break;
	case UART_RX_BUF_RELEASED:
		break;
	case UART_RX_STOPPED:
		break;
	default:
		break;
	}
}

static void uart_rtio_iodev_default_submit_sync(struct rtio_iodev_sqe *iodev_sqe)
{
	const struct device *dev = iodev_sqe->sqe.iodev->data;
	struct uart_rtio_sync_ctx sync_ctx;
	int err = 0;

	LOG_DBG("Sync RTIO work item for: %p", (void *)dev);

	k_sem_init(&sync_ctx.sem, 0, 1);
	sync_ctx.result = 0;
	sync_ctx.rx_received = 0;
	sync_ctx.for_rx = false;

	/* Set our callback for the duration of this operation */
	err = uart_callback_set(dev, uart_rtio_async_cb, &sync_ctx);
	if (err != 0) {
		LOG_ERR("Failed to set UART callback: %d", err);
		rtio_iodev_sqe_err(iodev_sqe, err);
		return;
	}

	/* Process linked transaction SQEs */
	struct rtio_iodev_sqe *txn_head = iodev_sqe;
	struct rtio_iodev_sqe *txn_curr = iodev_sqe;

	do {
		struct rtio_sqe *sqe = &txn_curr->sqe;

		switch (sqe->op) {
		case RTIO_OP_TX:
			err = uart_tx(dev, sqe->tx.buf, sqe->tx.buf_len, SYS_FOREVER_US);
			if (err == 0) {
				err = k_sem_take(&sync_ctx.sem, K_MSEC(5000));
				if (err == 0) {
					err = sync_ctx.result;
				}
			}
			break;
		case RTIO_OP_TINY_TX:
			err = uart_tx(dev, sqe->tiny_tx.buf, sqe->tiny_tx.buf_len,
				      SYS_FOREVER_US);
			if (err == 0) {
				err = k_sem_take(&sync_ctx.sem, K_MSEC(5000));
				if (err == 0) {
					err = sync_ctx.result;
				}
			}
			break;
		case RTIO_OP_RX:
			sync_ctx.rx_received = 0;
			sync_ctx.for_rx = true;
			err = uart_rx_enable(dev, sqe->rx.buf, sqe->rx.buf_len,
					     1000000);
			if (err == 0) {
				err = k_sem_take(&sync_ctx.sem, K_MSEC(5000));
				if (err == -EAGAIN) {
					/* Timed out — clean up RX DMA */
					uart_rx_disable(dev);
					/* Wait for RX_DISABLED callback */
					k_sem_take(&sync_ctx.sem, K_MSEC(1000));
				} else if (err == 0) {
					err = sync_ctx.result;
				}
			}
			sync_ctx.for_rx = false;
			break;
		default:
			LOG_ERR("Invalid op code %d for submission %p", sqe->op, (void *)sqe);
			err = -EIO;
			break;
		}

		if (err != 0) {
			break;
		}

		/* Reset for next op in transaction */
		k_sem_reset(&sync_ctx.sem);
		sync_ctx.result = 0;

		txn_curr = rtio_txn_next(txn_curr);
	} while (txn_curr != NULL);

	/* Clear the callback to prevent stale pointer access from
	 * loopback data or other asynchronous UART events after this
	 * stack frame is gone.
	 */
	uart_callback_set(dev, NULL, NULL);

	if (err != 0) {
		rtio_iodev_sqe_err(txn_head, err);
	} else {
		rtio_iodev_sqe_ok(txn_head, 0);
	}
}

void uart_rtio_iodev_default_submit(const struct device *dev,
				    struct rtio_iodev_sqe *iodev_sqe)
{
	LOG_DBG("Executing fallback for dev: %p, sqe: %p", (void *)dev, (void *)iodev_sqe);

	struct rtio_work_req *req = rtio_work_req_alloc();

	if (req == NULL) {
		LOG_ERR("RTIO work item allocation failed. Consider to increase "
			"CONFIG_RTIO_WORKQ_POOL_ITEMS.");
		rtio_iodev_sqe_err(iodev_sqe, -ENOMEM);
		return;
	}

	rtio_work_req_submit(req, iodev_sqe, uart_rtio_iodev_default_submit_sync);
}

static inline k_spinlock_key_t uart_spin_lock(struct uart_rtio *ctx)
{
	return k_spin_lock(&ctx->lock);
}

static inline void uart_spin_unlock(struct uart_rtio *ctx, k_spinlock_key_t key)
{
	k_spin_unlock(&ctx->lock, key);
}

void uart_rtio_init(struct uart_rtio *ctx, const struct device *dev)
{
	mpsc_init(&ctx->io_q);
	ctx->txn_head = NULL;
	ctx->txn_curr = NULL;
	ctx->dev = dev;
	ctx->iodev.data = (void *)dev;
	ctx->iodev.api = &uart_iodev_api;
}

static bool uart_rtio_next(struct uart_rtio *ctx, bool completion)
{
	k_spinlock_key_t key = uart_spin_lock(ctx);

	if (!completion && ctx->txn_curr != NULL) {
		uart_spin_unlock(ctx, key);
		return false;
	}

	struct mpsc_node *next = mpsc_pop(&ctx->io_q);

	if (next != NULL) {
		struct rtio_iodev_sqe *next_sqe = CONTAINER_OF(next, struct rtio_iodev_sqe, q);

		ctx->txn_head = next_sqe;
		ctx->txn_curr = next_sqe;
	} else {
		ctx->txn_head = NULL;
		ctx->txn_curr = NULL;
	}

	uart_spin_unlock(ctx, key);

	return (ctx->txn_curr != NULL);
}

bool uart_rtio_complete(struct uart_rtio *ctx, int status)
{
	struct rtio_iodev_sqe *txn_head = ctx->txn_head;
	bool result;

	result = uart_rtio_next(ctx, true);

	if (status < 0) {
		rtio_iodev_sqe_err(txn_head, status);
	} else {
		rtio_iodev_sqe_ok(txn_head, status);
	}

	return result;
}

bool uart_rtio_submit(struct uart_rtio *ctx, struct rtio_iodev_sqe *iodev_sqe)
{
	mpsc_push(&ctx->io_q, &iodev_sqe->q);
	return uart_rtio_next(ctx, false);
}
