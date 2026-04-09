/*
 * Copyright (c) 2025 iWorx Systems Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nuvoton_numaker_timer

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/clock_control_numaker.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <zephyr/dt-bindings/counter/numaker-timer.h>

#include <NuMicro.h>

LOG_MODULE_REGISTER(counter_numaker_timer);

/* Timer register offsets */
#define TMR_CTL    0x00
#define TMR_CMP    0x04
#define TMR_INTSTS 0x08
#define TMR_CNT    0x0C

/* CTL register bits */
#define TMR_CTL_PSC_Pos 0
#define TMR_CTL_INTEN   BIT(29)
#define TMR_CTL_CNTEN   BIT(30)

/* OPMODE values (bits 28:27) */
#define TMR_OPMODE_PERIODIC (0x1UL << 27)
#define TMR_OPMODE_TOGGLE   (0x2UL << 27)

/* CTL register: event counter bit */
#define TMR_CTL_EXTCNTEN  BIT(24)

/* EXTCTL register offset and bits */
#define TMR_EXTCTL            0x14
#define TMR_EXTCTL_CNTPHASE   BIT(0)

/* TRGCTL register offset and bits */
#define TMR_TRGCTL            0x1C
#define TMR_TRGCTL_TRGSSEL    BIT(0)
#define TMR_TRGCTL_TRGPDMA    BIT(4)

/* INTSTS register bits */
#define TMR_INTSTS_TIF BIT(0)

/* CMP register: 24-bit max value. Note: never write 0 or 1 to CMPDAT. */
#define TMR_CMP_MAX 0x00FFFFFFUL

/* CNT register mask */
#define TMR_CNT_Msk 0x00FFFFFFUL

/**
 * Get the input clock frequency for a timer module.
 * Implements the same logic as BSP TIMER_GetModuleClock() inline.
 */
static uint32_t numaker_timer_get_clock(TIMER_T *timer)
{
	uint32_t src;
	const uint32_t clk_table[] = {__HXT, __LXT, 0UL, 0UL, 0UL, __LIRC, 0UL, __HIRC};

	if (timer == TIMER0) {
		src = (CLK->CLKSEL1 & CLK_CLKSEL1_TMR0SEL_Msk) >> CLK_CLKSEL1_TMR0SEL_Pos;
	} else if (timer == TIMER1) {
		src = (CLK->CLKSEL1 & CLK_CLKSEL1_TMR1SEL_Msk) >> CLK_CLKSEL1_TMR1SEL_Pos;
	} else if (timer == TIMER2) {
		src = (CLK->CLKSEL1 & CLK_CLKSEL1_TMR2SEL_Msk) >> CLK_CLKSEL1_TMR2SEL_Pos;
	} else {
		src = (CLK->CLKSEL1 & CLK_CLKSEL1_TMR3SEL_Msk) >> CLK_CLKSEL1_TMR3SEL_Pos;
	}

	if (src == 2UL) {
		if (timer == TIMER0 || timer == TIMER1) {
			return CLK_GetPCLK0Freq();
		}
		return CLK_GetPCLK1Freq();
	}

	return clk_table[src];
}

struct counter_numaker_data {
	counter_top_callback_t top_cb;
	void *top_user_data;
	uint32_t guard_period;
	uint32_t freq;
	bool pinctrl_applied;
};

struct counter_numaker_config {
	struct counter_config_info counter_info;
	TIMER_T *base;
	uint32_t clk_modidx;
	uint32_t clk_src;
	uint32_t clk_div;
	const struct device *clk_dev;
	struct reset_dt_spec reset;
	const struct pinctrl_dev_config *pincfg;
	uint8_t prescaler;
	uint8_t timer_mode;
	uint8_t trigger_source;
	bool count_rising;
	bool trigger_pdma;
	void (*irq_config)(const struct device *dev);
};

static inline uint32_t tmr_read(const struct counter_numaker_config *cfg, uint32_t offset)
{
	return sys_read32((mem_addr_t)cfg->base + offset);
}

static inline void tmr_write(const struct counter_numaker_config *cfg, uint32_t offset,
			     uint32_t val)
{
	sys_write32(val, (mem_addr_t)cfg->base + offset);
}

static int counter_numaker_start(const struct device *dev)
{
	const struct counter_numaker_config *cfg = dev->config;
	struct counter_numaker_data *data = dev->data;
	uint32_t ctl;

	/* Apply pinctrl on first start so pins stay as GPIO until needed */
	if (!data->pinctrl_applied && cfg->pincfg != NULL) {
		int err = pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_DEFAULT);

		if (err != 0) {
			return err;
		}
		data->pinctrl_applied = true;
	}

	/* Enable PDMA trigger if configured */
	if (cfg->trigger_pdma) {
		uint32_t trgctl = TMR_TRGCTL_TRGPDMA;

		if (cfg->trigger_source == NUMAKER_TIMER_TRIGGER_SRC_CAPTURE) {
			trgctl |= TMR_TRGCTL_TRGSSEL;
		}
		tmr_write(cfg, TMR_TRGCTL, trgctl);
	}

	ctl = tmr_read(cfg, TMR_CTL);
	ctl |= TMR_CTL_CNTEN;
	tmr_write(cfg, TMR_CTL, ctl);

	return 0;
}

static int counter_numaker_stop(const struct device *dev)
{
	const struct counter_numaker_config *cfg = dev->config;
	uint32_t ctl = tmr_read(cfg, TMR_CTL);

	ctl &= ~TMR_CTL_CNTEN;
	tmr_write(cfg, TMR_CTL, ctl);

	/* Disable PDMA trigger */
	if (cfg->trigger_pdma) {
		tmr_write(cfg, TMR_TRGCTL, 0);
	}

	return 0;
}

static int counter_numaker_get_value(const struct device *dev, uint32_t *ticks)
{
	const struct counter_numaker_config *cfg = dev->config;

	*ticks = tmr_read(cfg, TMR_CNT) & TMR_CNT_Msk;

	return 0;
}

static uint32_t counter_numaker_get_top_value(const struct device *dev)
{
	const struct counter_numaker_config *cfg = dev->config;

	return tmr_read(cfg, TMR_CMP) & TMR_CMP_MAX;
}

static uint32_t counter_numaker_get_freq(const struct device *dev)
{
	struct counter_numaker_data *data = dev->data;

	return data->freq;
}

static uint32_t counter_numaker_get_guard_period(const struct device *dev, uint32_t flags)
{
	struct counter_numaker_data *data = dev->data;

	return data->guard_period;
}

static int counter_numaker_set_guard_period(const struct device *dev, uint32_t guard,
					    uint32_t flags)
{
	struct counter_numaker_data *data = dev->data;

	__ASSERT_NO_MSG(guard < counter_numaker_get_top_value(dev));

	data->guard_period = guard;
	return 0;
}

static uint32_t counter_numaker_get_pending_int(const struct device *dev)
{
	const struct counter_numaker_config *cfg = dev->config;

	return !!(tmr_read(cfg, TMR_INTSTS) & TMR_INTSTS_TIF);
}

static int counter_numaker_set_top_value(const struct device *dev,
					 const struct counter_top_cfg *top_cfg)
{
	const struct counter_numaker_config *cfg = dev->config;
	struct counter_numaker_data *data = dev->data;
	uint32_t ctl;
	int err = 0;

	/* Disable interrupt while reconfiguring */
	ctl = tmr_read(cfg, TMR_CTL);
	ctl &= ~TMR_CTL_INTEN;
	tmr_write(cfg, TMR_CTL, ctl);

	/* Set compare value (periodic mode: counter resets at CMP match) */
	tmr_write(cfg, TMR_CMP, top_cfg->ticks & TMR_CMP_MAX);

	/* Clear pending interrupt */
	tmr_write(cfg, TMR_INTSTS, TMR_INTSTS_TIF);

	data->top_cb = top_cfg->callback;
	data->top_user_data = top_cfg->user_data;

	if (!(top_cfg->flags & COUNTER_TOP_CFG_DONT_RESET)) {
		/* Write any value to CNT resets it to 0 */
		tmr_write(cfg, TMR_CNT, 0);
	} else if ((tmr_read(cfg, TMR_CNT) & TMR_CNT_Msk) >= top_cfg->ticks) {
		err = -ETIME;
		if (top_cfg->flags & COUNTER_TOP_CFG_RESET_WHEN_LATE) {
			tmr_write(cfg, TMR_CNT, 0);
		}
	}

	if (top_cfg->callback || cfg->trigger_pdma) {
		ctl = tmr_read(cfg, TMR_CTL);
		ctl |= TMR_CTL_INTEN;
		tmr_write(cfg, TMR_CTL, ctl);
	}

	return err;
}

static int counter_numaker_set_alarm(const struct device *dev, uint8_t chan,
				     const struct counter_alarm_cfg *alarm_cfg)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(chan);
	ARG_UNUSED(alarm_cfg);

	return -ENOTSUP;
}

static int counter_numaker_cancel_alarm(const struct device *dev, uint8_t chan)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(chan);

	return -ENOTSUP;
}

static void counter_numaker_isr(const struct device *dev)
{
	const struct counter_numaker_config *cfg = dev->config;
	struct counter_numaker_data *data = dev->data;
	uint32_t intsts = tmr_read(cfg, TMR_INTSTS);

	if (!(intsts & TMR_INTSTS_TIF)) {
		return;
	}

	/* Clear interrupt flag */
	tmr_write(cfg, TMR_INTSTS, TMR_INTSTS_TIF);

	if (data->top_cb) {
		data->top_cb(dev, data->top_user_data);
	}
}

static int counter_numaker_init(const struct device *dev)
{
	const struct counter_numaker_config *cfg = dev->config;
	struct counter_numaker_data *data = dev->data;
	struct numaker_scc_subsys scc_subsys;
	int err;

	SYS_UnlockReg();

	/* Enable module clock */
	memset(&scc_subsys, 0x00, sizeof(scc_subsys));
	scc_subsys.subsys_id = NUMAKER_SCC_SUBSYS_ID_PCC;
	scc_subsys.pcc.clk_modidx = cfg->clk_modidx;
	scc_subsys.pcc.clk_src = cfg->clk_src;
	scc_subsys.pcc.clk_div = cfg->clk_div;

	err = clock_control_on(cfg->clk_dev, (clock_control_subsys_t)&scc_subsys);
	if (err != 0) {
		goto done;
	}

	err = clock_control_configure(cfg->clk_dev, (clock_control_subsys_t)&scc_subsys, NULL);
	if (err != 0) {
		goto done;
	}

	data->freq = numaker_timer_get_clock((TIMER_T *)cfg->base) / (cfg->prescaler + 1U);

	/* Reset the timer */
	if (device_is_ready(cfg->reset.dev)) {
		reset_line_toggle_dt(&cfg->reset);
	}

	/* Build TIMER_CTL value based on inferred timer mode */
	uint32_t ctl_val = (uint32_t)cfg->prescaler << TMR_CTL_PSC_Pos;

	switch (cfg->timer_mode) {
	case NUMAKER_TIMER_MODE_EVENT_COUNTER:
		/* Hardware event counter on TMx pin */
		ctl_val |= TMR_OPMODE_PERIODIC | TMR_CTL_EXTCNTEN;
		tmr_write(cfg, TMR_EXTCTL,
			  (cfg->count_rising ? TMR_EXTCTL_CNTPHASE : 0));
		data->freq = 0;
		break;
	case NUMAKER_TIMER_MODE_TOGGLE_OUTPUT:
		ctl_val |= TMR_OPMODE_TOGGLE;
		break;
	default: /* NUMAKER_TIMER_MODE_PERIODIC */
		ctl_val |= TMR_OPMODE_PERIODIC;
		break;
	}

	tmr_write(cfg, TMR_CTL, ctl_val);

	/* Set compare to max (default top) */
	tmr_write(cfg, TMR_CMP, cfg->counter_info.max_top_value);

	/* Clear any pending interrupts */
	tmr_write(cfg, TMR_INTSTS, TMR_INTSTS_TIF);

	/* Configure IRQ */
	cfg->irq_config(dev);

done:
	SYS_LockReg();
	return err;
}

static DEVICE_API(counter, counter_numaker_api) = {
	.start = counter_numaker_start,
	.stop = counter_numaker_stop,
	.get_value = counter_numaker_get_value,
	.set_alarm = counter_numaker_set_alarm,
	.cancel_alarm = counter_numaker_cancel_alarm,
	.set_top_value = counter_numaker_set_top_value,
	.get_pending_int = counter_numaker_get_pending_int,
	.get_top_value = counter_numaker_get_top_value,
	.get_guard_period = counter_numaker_get_guard_period,
	.set_guard_period = counter_numaker_set_guard_period,
	.get_freq = counter_numaker_get_freq,
};

#define COUNTER_NUMAKER_IRQ_CONFIG(n)                                                              \
	static void irq_config_##n(const struct device *dev)                                       \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), counter_numaker_isr,        \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}

#define COUNTER_NUMAKER_PINCTRL_DEFINE(n)                                                          \
	COND_CODE_1(DT_INST_PINCTRL_HAS_NAME(n, default),                                         \
		    (PINCTRL_DT_INST_DEFINE(n);),                                                  \
		    ())

#define COUNTER_NUMAKER_PINCTRL_INIT(n)                                                            \
	COND_CODE_1(DT_INST_PINCTRL_HAS_NAME(n, default),                                         \
		    (.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),),                                \
		    (.pincfg = NULL,))

#define COUNTER_NUMAKER_COUNT_RISING(n)                                                            \
	(DT_INST_ENUM_IDX_OR(n, count_edge, NUMAKER_TIMER_COUNT_EDGE_RISING)                       \
	 == NUMAKER_TIMER_COUNT_EDGE_RISING)

/* Infer timer mode from which optional properties are present */
#define COUNTER_NUMAKER_TIMER_MODE(n)                                                               \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(n, count_edge),                                          \
		    (NUMAKER_TIMER_MODE_EVENT_COUNTER),                                             \
		    (COND_CODE_1(DT_INST_PROP(n, output_clk),                                        \
				  (NUMAKER_TIMER_MODE_TOGGLE_OUTPUT),                                  \
				  (NUMAKER_TIMER_MODE_PERIODIC))))

#define COUNTER_NUMAKER_INIT(n)                                                                    \
	COUNTER_NUMAKER_IRQ_CONFIG(n)                                                              \
	COUNTER_NUMAKER_PINCTRL_DEFINE(n)                                                          \
	static struct counter_numaker_data counter_numaker_data_##n;                               \
	static const struct counter_numaker_config counter_numaker_cfg_##n = {                     \
		.counter_info =                                                                    \
			{                                                                          \
				.max_top_value = TMR_CMP_MAX,                                      \
				.flags = COUNTER_CONFIG_INFO_COUNT_UP,                             \
				.freq = 0,                                                         \
				.channels = 0,                                                     \
			},                                                                         \
		.base = (TIMER_T *)DT_INST_REG_ADDR(n),                                            \
		.clk_modidx = DT_INST_CLOCKS_CELL(n, clock_module_index),                          \
		.clk_src = DT_INST_CLOCKS_CELL(n, clock_source),                                   \
		.clk_div = DT_INST_CLOCKS_CELL(n, clock_divider),                                  \
		.clk_dev = DEVICE_DT_GET(DT_PARENT(DT_INST_CLOCKS_CTLR(n))),                       \
		.reset = RESET_DT_SPEC_INST_GET(n),                                                \
		COUNTER_NUMAKER_PINCTRL_INIT(n)                                                    \
		.prescaler = DT_INST_PROP(n, prescaler),                                           \
		.timer_mode = COUNTER_NUMAKER_TIMER_MODE(n),                                        \
		.trigger_source = DT_INST_ENUM_IDX_OR(n, trigger_source,                             \
						      NUMAKER_TIMER_TRIGGER_SRC_TIMEOUT),           \
		.count_rising = COUNTER_NUMAKER_COUNT_RISING(n),                                   \
		.trigger_pdma = DT_INST_PROP(n, trigger_pdma),                                     \
		.irq_config = irq_config_##n,                                                      \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, counter_numaker_init, NULL, &counter_numaker_data_##n,            \
			      &counter_numaker_cfg_##n, PRE_KERNEL_1,                              \
			      CONFIG_COUNTER_INIT_PRIORITY, &counter_numaker_api);

DT_INST_FOREACH_STATUS_OKAY(COUNTER_NUMAKER_INIT)
