/*
 * Copyright (c) 2024 iWorx Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_UART_RTIO_H_
#define ZEPHYR_DRIVERS_UART_RTIO_H_

#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/rtio/rtio.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Driver context for implementing UART with RTIO
 */
struct uart_rtio {
	struct k_spinlock lock;
	struct rtio *r;
	struct mpsc io_q;
	struct rtio_iodev iodev;
	struct rtio_iodev_sqe *txn_head;
	struct rtio_iodev_sqe *txn_curr;
	const struct device *dev;
};

/**
 * @brief Statically define a uart_rtio context
 *
 * @param _name Symbolic name of the context
 * @param _sq_sz Submission queue entry pool size
 * @param _cq_sz Completion queue entry pool size
 */
#define UART_RTIO_DEFINE(_name, _sq_sz, _cq_sz)			\
	RTIO_DEFINE(CONCAT(_name, _r), _sq_sz, _cq_sz);		\
	static struct uart_rtio _name = {				\
		.r = &CONCAT(_name, _r),				\
	};

/**
 * @brief Initialize a UART RTIO context
 *
 * @param ctx UART RTIO driver context
 * @param dev UART device
 */
void uart_rtio_init(struct uart_rtio *ctx, const struct device *dev);

/**
 * @brief Signal that the current (ctx->txn_curr) submission has been completed
 *
 * @param ctx UART RTIO driver context
 * @param status Completion status, negative values are errors
 *
 * @retval true Next submission is ready to start
 * @retval false No more submissions to work on
 */
bool uart_rtio_complete(struct uart_rtio *ctx, int status);

/**
 * @brief Submit, atomically, a submission to work on at some point
 *
 * @retval true Next submission is ready to start
 * @retval false No new submission to start or submissions are in progress already
 */
bool uart_rtio_submit(struct uart_rtio *ctx, struct rtio_iodev_sqe *iodev_sqe);

/**
 * @brief Fallback UART RTIO submit implementation.
 *
 * Default RTIO UART implementation for drivers that use the async API
 * underneath. Converts RTIO SQEs to uart_tx / uart_rx_enable calls via
 * the RTIO workqueue.
 */
void uart_rtio_iodev_default_submit(const struct device *dev,
				    struct rtio_iodev_sqe *iodev_sqe);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_UART_RTIO_H_ */
