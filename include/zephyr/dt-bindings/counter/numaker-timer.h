/*
 * Copyright (c) 2025 iWorx Systems Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_COUNTER_NUMAKER_TIMER_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_COUNTER_NUMAKER_TIMER_H_

/** Timer operating mode (timer-mode property) */
#define NUMAKER_TIMER_MODE_PERIODIC      0
#define NUMAKER_TIMER_MODE_EVENT_COUNTER 1
#define NUMAKER_TIMER_MODE_TOGGLE_OUTPUT 2

/** Count edge selection (count-edge property) */
#define NUMAKER_TIMER_COUNT_EDGE_RISING  0
#define NUMAKER_TIMER_COUNT_EDGE_FALLING 1

/** PDMA trigger source selection (trigger-source property) */
#define NUMAKER_TIMER_TRIGGER_SRC_TIMEOUT 0
#define NUMAKER_TIMER_TRIGGER_SRC_CAPTURE 1

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_COUNTER_NUMAKER_TIMER_H_ */
