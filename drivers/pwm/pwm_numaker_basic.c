/*
 * Copyright (c) 2023 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nuvoton_numaker_bpwm

#include <zephyr/kernel.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/clock_control_numaker.h>
#include <zephyr/logging/log.h>
#include <soc.h>
#include <NuMicro.h>

LOG_MODULE_REGISTER(bpwm_numaker, CONFIG_PWM_LOG_LEVEL);

/* 11-bit prescaler in Numaker BPWM modules */
#define NUMAKER_PWM_MAX_PRESCALER BIT(11)
#define NUMAKER_PWM_CHANNEL_COUNT 6
#define NUMAKER_PWM_RELOAD_CNT    (0xFFFFU)
#define NUMAKER_SYSCLK_FREQ       DT_PROP(DT_NODELABEL(sysclk), clock_frequency)
/* BPWM channel 0~5 mask */
#define NUMAKER_PWM_CHANNEL_MASK  (0x3FU)

/* Device config */
struct pwm_numaker_basic_config {
	/* BPWM base address */
	BPWM_T *bpwm;
	uint32_t prescale;
	const struct reset_dt_spec reset;
	/* clock configuration */
	uint32_t clk_modidx;
	uint32_t clk_src;
	uint32_t clk_div;
	const struct device *clk_dev;
	const struct pinctrl_dev_config *pincfg;
	void (*irq_config_func)(const struct device *dev);
};

struct pwm_numaker_basic_capture_data {
	pwm_capture_callback_handler_t callback;
	void *user_data;
	/* Only support either one of PWM_CAPTURE_TYPE_PULSE, PWM_CAPTURE_TYPE_PERIOD */
	bool pulse_capture;
	bool single_mode;
	bool is_busy;
	uint32_t curr_edge_mode;
	uint32_t next_edge_mode;
};

/* Driver context/data */
struct pwm_numaker_basic_data {
	uint32_t clock_freq;
	uint32_t cycles_per_sec;
#ifdef CONFIG_PWM_CAPTURE
	uint32_t overflows;
	struct pwm_numaker_capture_data capture[NUMAKER_PWM_CHANNEL_COUNT];
#endif /* CONFIG_PWM_CAPTURE */
};

static void pwm_numaker_basic_configure(const struct device *dev)
{
	const struct pwm_numaker_basic_config *cfg = dev->config;
	BPWM_T *bpwm = cfg->bpwm;

	/* Disable BPWM channel 0~5 before config */
	BPWM_ForceStop(bpwm, NUMAKER_PWM_CHANNEL_MASK);

	/* Set BPWM default normal polarity as inverse disabled */
	bpwm->POLCTL &= ~(NUMAKER_PWM_CHANNEL_MASK << BPWM_POLCTL_PINV0_Pos);
}

/* PWM api functions */
static int pwm_numaker_basic_set_cycles(const struct device *dev, uint32_t channel,
				  uint32_t period_cycles, uint32_t pulse_cycles, pwm_flags_t flags)
{
	const struct pwm_numaker_basic_config *cfg = dev->config;
	struct pwm_numaker_basic_data *data = dev->data;
	BPWM_T *bpwm = cfg->bpwm;
	uint32_t channel_mask = BIT(channel);

	LOG_DBG("Channel=0x%x, CAPIEN=0x%x, CAPIF=0x%x", channel, bpwm->CAPIEN,
		bpwm->CAPIF);

	/* Set BPWM polarity */
	if (flags & PWM_POLARITY_INVERTED) {
		bpwm->POLCTL |= BIT(BPWM_POLCTL_PINV0_Pos + channel);
	} else {
		bpwm->POLCTL &= ~BIT(BPWM_POLCTL_PINV0_Pos + channel);
	}

	/* Force disable BPWM channel as while pulse_cycles = 0 */
	if (period_cycles == 0U) {
		BPWM_Stop(bpwm, channel_mask);
		BPWM_ForceStop(bpwm, channel_mask);
		BPWM_DisableOutput(bpwm, channel_mask);
		return 0;
	}

	/* Set BPWM channel & output configuration */
	BPWM_ConfigOutputChannel(bpwm, channel, data->cycles_per_sec / period_cycles,
				 (10000U * pulse_cycles) / period_cycles);

	/* Enable BPWM Output path for BPWM channel */
	BPWM_EnableOutput(bpwm, channel_mask);

	/* Enable Timer for BPWM channel */
	BPWM_Start(bpwm, channel_mask);

	LOG_DBG("cycles_per_sec=0x%x, pulse_cycles=0x%x, period_cycles=0x%x",
		data->cycles_per_sec, pulse_cycles, period_cycles);
	LOG_DBG("CTL1=0x%x, POEN=0x%x, CNTEN=0x%x", bpwm->CTL1, bpwm->POEN, bpwm->CNTEN);
	LOG_DBG("Channel=0x%x, CAPIEN=0x%x, CAPIF=0x%x", channel, bpwm->CAPIEN, bpwm->CAPIF);

	return 0;
}

static int pwm_numaker_basic_get_cycles_per_sec(const struct device *dev, uint32_t channel,
					  uint64_t *cycles)
{
	const struct pwm_numaker_basic_config *cfg = dev->config;
	struct pwm_numaker_basic_data *data = dev->data;

	ARG_UNUSED(channel);

	data->cycles_per_sec = data->clock_freq / (cfg->prescale + 1U);
	*cycles = (uint64_t)data->cycles_per_sec;

	return 0;
}

#ifdef CONFIG_PWM_CAPTURE
static int pwm_numaker_basic_configure_capture(const struct device *dev, uint32_t channel,
					 pwm_flags_t flags, pwm_capture_callback_handler_t cb,
					 void *user_data)
{
	struct pwm_numaker_basic_data *data = dev->data;
	uint32_t pair = channel;

	LOG_DBG("");

	data->capture[pair].callback = cb;
	data->capture[pair].user_data = user_data;

	if (data->capture[pair].is_busy) {
		LOG_ERR("Capture already active on this channel %d", pair);
		return -EBUSY;
	}
	if ((flags & PWM_CAPTURE_TYPE_MASK) == PWM_CAPTURE_TYPE_BOTH) {
		LOG_ERR("Cannot capture both period and pulse width");
		return -ENOTSUP;
	}

	if ((flags & PWM_CAPTURE_MODE_MASK) == PWM_CAPTURE_MODE_CONTINUOUS) {
		data->capture[pair].single_mode = false;
	} else {
		data->capture[pair].single_mode = true;
	}

	if (flags & PWM_CAPTURE_TYPE_PERIOD) {
		data->capture[pair].pulse_capture = false;

		if (flags & PWM_POLARITY_INVERTED) {
			data->capture[pair].curr_edge_mode = BPWM_CAPTURE_INT_FALLING_LATCH;
			data->capture[pair].next_edge_mode = BPWM_CAPTURE_INT_FALLING_LATCH;
		} else {
			data->capture[pair].curr_edge_mode = BPWM_CAPTURE_INT_RISING_LATCH;
			data->capture[pair].next_edge_mode = BPWM_CAPTURE_INT_RISING_LATCH;
		}
	} else {
		data->capture[pair].pulse_capture = true;

		if (flags & PWM_POLARITY_INVERTED) {
			data->capture[pair].curr_edge_mode = BPWM_CAPTURE_INT_FALLING_LATCH;
			data->capture[pair].next_edge_mode = BPWM_CAPTURE_INT_RISING_LATCH;
		} else {
			data->capture[pair].curr_edge_mode = BPWM_CAPTURE_INT_RISING_LATCH;
			data->capture[pair].next_edge_mode = BPWM_CAPTURE_INT_FALLING_LATCH;
		}
	}

	return 0;
}

static int pwm_numaker_enable_capture(const struct device *dev, uint32_t channel)
{
	const struct pwm_numaker_basic_config *cfg = dev->config;
	struct pwm_numaker_basic_data *data = dev->data;
	BPWM_T *bpwm = cfg->bpwm;
	uint32_t pair = channel;
	uint32_t channel_mask = BIT(channel);
	uint32_t unit_time_nsec = (1000000000U / data->cycles_per_sec);

	LOG_DBG("");

	if (!data->capture[pair].callback) {
		LOG_ERR("PWM capture not configured");
		return -EINVAL;
	}

	if (data->capture[pair].is_busy) {
		LOG_ERR("Capture already active on this channel %d", pair);
		return -EBUSY;
	}

	data->capture[pair].is_busy = true;

	/* Set capture configuration */
	BPWM_ConfigCaptureChannel(bpwm, channel, unit_time_nsec, 0);

	/* Enable Capture Function for BPWM */
	BPWM_EnableCapture(bpwm, channel_mask);

	/* Enable Timer for BPWM */
	BPWM_Start(bpwm, channel_mask);

	BPWM_ClearCaptureIntFlag(bpwm, channel,
				 BPWM_CAPTURE_INT_FALLING_LATCH | BPWM_CAPTURE_INT_RISING_LATCH);

	/* EnableInterrupt */
	BPWM_EnableCaptureInt(bpwm, channel, data->capture[pair].curr_edge_mode);

	LOG_DBG("Channel=0x%x, CAPIEN=0x%x, CAPIF=0x%x", channel, bpwm->CAPIEN,
		bpwm->CAPIF);

	return 0;
}

static int pwm_numaker_disable_capture(const struct device *dev, uint32_t channel)
{
	const struct pwm_numaker_basic_config *cfg = dev->config;
	struct pwm_numaker_basic_data *data = dev->data;
	BPWM_T *bpwm = cfg->bpwm;
	uint32_t channel_mask = BIT(channel);

	LOG_DBG("");

	data->capture[channel].is_busy = false;
	BPWM_Stop(bpwm, channel_mask);
	BPWM_ForceStop(bpwm, channel_mask);
	BPWM_DisableCapture(bpwm, channel_mask);
	BPWM_DisableCaptureInt(bpwm, channel,
			       BPWM_CAPTURE_INT_RISING_LATCH | BPWM_CAPTURE_INT_FALLING_LATCH);
	BPWM_ClearCaptureIntFlag(bpwm, channel,
				 BPWM_CAPTURE_INT_FALLING_LATCH | BPWM_CAPTURE_INT_RISING_LATCH);
	LOG_DBG("CAPIEN = 0x%x\n", bpwm->CAPIEN);
	return 0;
}

/*
 * Get capture cycles between current channel edge until next chnannel edge.
 * The capture period counter down count from 0x10000, and auto-reload to 0x10000
 */
static int pwm_numaker_get_cap_cycle(BPWM_T *bpwm, uint32_t channel, uint32_t curr_edge,
				       uint32_t next_edge, uint32_t *cycles)
{
	uint16_t curr_cnt;
	uint16_t next_cnt;
	uint32_t next_if_mask;
	uint32_t capif_base;
	uint32_t time_out_cnt;
	int status = 0;
	uint32_t period_reloads = 0;

	/* PWM counter is timing critical, to avoid print msg from irq_isr until getting cycles */
	LOG_DBG("");

	BPWM_ClearPeriodIntFlag(bpwm, channel);

	capif_base = (next_edge == BPWM_CAPTURE_INT_FALLING_LATCH) ? BPWM_CAPIF_CFLIF0_Pos
								  : BPWM_CAPIF_CRLIF0_Pos;
	next_if_mask = BIT(capif_base + channel);
	time_out_cnt = NUMAKER_SYSCLK_FREQ / 2; /* 500 ms time-out */
	LOG_DBG("Channel=0x%x, R-Cnt=0x%x, F-Cnt0x%x, CNT-0x%x", channel,
		BPWM_GET_CAPTURE_RISING_DATA(bpwm, channel),
		BPWM_GET_CAPTURE_FALLING_DATA(bpwm, channel), bpwm->CNT[channel]);
	curr_cnt = (curr_edge == BPWM_CAPTURE_INT_FALLING_LATCH)
			  ? BPWM_GET_CAPTURE_FALLING_DATA(bpwm, channel)
			  : (uint16_t)BPWM_GET_CAPTURE_RISING_DATA(bpwm, channel);

	/* Wait for Capture Next Indicator */
	while ((bpwm->CAPIF & next_if_mask) == 0) {
		if (BPWM_GetPeriodIntFlag(bpwm, channel)) {
			BPWM_ClearPeriodIntFlag(bpwm, channel);
			period_reloads++;
		}
		if (--time_out_cnt == 0) {
			status = -EAGAIN;
			goto done;
		}
	}

	/* Clear Capture Falling and Rising Indicator */
	BPWM_ClearCaptureIntFlag(bpwm, channel,
				 BPWM_CAPTURE_INT_FALLING_LATCH | BPWM_CAPTURE_INT_RISING_LATCH);

	/* Get Capture Latch Counter Data */
	next_cnt = (next_edge == BPWM_CAPTURE_INT_FALLING_LATCH)
			  ? (uint16_t)BPWM_GET_CAPTURE_FALLING_DATA(bpwm, channel)
			  : (uint16_t)BPWM_GET_CAPTURE_RISING_DATA(bpwm, channel);

	*cycles = (period_reloads * NUMAKER_PWM_RELOAD_CNT) + curr_cnt - next_cnt;
	LOG_DBG("cycles=0x%x, period_reloads=0x%x, CAPIF=0x%x, cur-0x%x ,next-0x%x",
		*cycles, period_reloads, bpwm->CAPIF, curr_cnt, next_cnt);

done:
	return status;
}

static void pwm_numaker_channel_cap(const struct device *dev, BPWM_T *bpwm, uint32_t channel)
{
	struct pwm_numaker_basic_data *data = dev->data;
	struct pwm_numaker_capture_data *capture;
	uint32_t cycles = 0;
	int status;

	BPWM_DisableCaptureInt(bpwm, channel, BPWM_CAPTURE_INT_RISING_LATCH |
				BPWM_CAPTURE_INT_FALLING_LATCH);

	capture = &data->capture[channel];

	/* Calculate cycles */
	status = pwm_numaker_get_cap_cycle(
		bpwm, channel, data->capture[channel].curr_edge_mode,
		data->capture[channel].next_edge_mode, &cycles);
	if (capture->pulse_capture) {
		/* For PWM_CAPTURE_TYPE_PULSE */
		capture->callback(dev, channel, 0, cycles, status, capture->user_data);
	} else {
		/* For PWM_CAPTURE_TYPE_PERIOD */
		capture->callback(dev, channel, cycles, 0, status, capture->user_data);
	}

	if (capture->single_mode) {
		BPWM_DisableCaptureInt(bpwm, channel, BPWM_CAPTURE_INT_RISING_LATCH |
					BPWM_CAPTURE_INT_FALLING_LATCH);
		data->capture[channel].is_busy = false;
	} else {
		BPWM_ClearCaptureIntFlag(bpwm, channel, BPWM_CAPTURE_INT_FALLING_LATCH |
					BPWM_CAPTURE_INT_RISING_LATCH);
		BPWM_EnableCaptureInt(bpwm, channel, data->capture[channel].curr_edge_mode);
		/* data->capture[channel].is_busy = true; */
	}
}

static void pwm_numaker_isr(const struct device *dev, uint32_t st_channel, uint32_t end_channel)
{
	const struct pwm_numaker_basic_config *cfg = dev->config;
	struct pwm_numaker_basic_data *data = dev->data;
	BPWM_T *bpwm = cfg->bpwm;
	struct pwm_numaker_capture_data *capture;
	uint32_t int_status;
	uint32_t cap_intsts;
	int i;
	uint32_t int_mask = (BIT(st_channel) | BIT(end_channel));
	uint32_t cap_int_rise_mask, cap_int_fall_mask;
	uint32_t cap_int_mask =
		(BPWM_CAPIF_CFLIF0_Msk << st_channel | BPWM_CAPIF_CRLIF0_Msk << st_channel |
		 BPWM_CAPIF_CFLIF0_Msk << end_channel | BPWM_CAPIF_CRLIF0_Msk << end_channel);

	/* Get Output int status */
	int_status = bpwm->AINTSTS & int_mask;
	/* Clear Output int status */
	if (int_status != 0x00) {
		bpwm->AINTSTS = int_status;
	}

	/* Get CAP int status */
	cap_intsts = bpwm->CAPIF & cap_int_mask;

	/* PWM counter is timing critical, to avoid print msg from irq_isr
	 *  until getting capture cycles.
	 */
	LOG_DBG("Channel=0x%x, CAPIEN=0x%x, CAPIF=0x%x, capIntMask=0x%x",
		st_channel, bpwm->CAPIEN, bpwm->CAPIF, cap_int_mask);
	if (cap_intsts != 0x00) { /* Capture Interrupt */
		/* Clear CAP int status */
		bpwm->CAPIF = cap_intsts;

		/* Rising latch or Falling latch */
		for (i = st_channel; i <= end_channel; i++) {
			capture = &data->capture[i];
			if (capture == NULL) {
				continue;
			}
			cap_int_rise_mask = (BPWM_CAPTURE_INT_RISING_LATCH << i);
			cap_int_fall_mask = (BPWM_CAPTURE_INT_FALLING_LATCH << i);
			if ((cap_int_rise_mask | cap_int_fall_mask) & cap_intsts) {
				pwm_numaker_channel_cap(dev, bpwm, i);
			}
		}
	}
}

static void pwm_numaker_p0_isr(const struct device *dev)
{
	/* Pair0 service channel 0, 1 */
	pwm_numaker_isr(dev, 0, 1);
}

static void pwm_numaker_p1_isr(const struct device *dev)
{
	/* Pair1 service channel 2, 3 */
	pwm_numaker_isr(dev, 2, 3);
}

static void pwm_numaker_p2_isr(const struct device *dev)
{
	/* Pair2 service channel 4, 5 */
	pwm_numaker_isr(dev, 4, 5);
}
#endif /* CONFIG_PWM_CAPTURE */

/* PWM driver registration */
static DEVICE_API(pwm, pwm_numaker_driver_api) = {
	.set_cycles = pwm_numaker_basic_set_cycles,
	.get_cycles_per_sec = pwm_numaker_basic_get_cycles_per_sec,
#ifdef CONFIG_PWM_CAPTURE
	.configure_capture = pwm_numaker_basic_configure_capture,
	.enable_capture = pwm_numaker_enable_capture,
	.disable_capture = pwm_numaker_disable_capture,
#endif /* CONFIG_PWM_CAPTURE */
};

/* Alternative BPWM clock get rate before support standard clock_control_get_rate */
static int pwm_numaker_basic_clk_get_rate(BPWM_T *bpwm, uint32_t *rate)
{
	uint32_t clk_src;
	uint32_t bpwm_clk_src;

	if (bpwm == BPWM0) {
		clk_src = CLK->CLKSEL2 & CLK_CLKSEL2_BPWM0SEL_Msk;
	} else if (bpwm == BPWM1) {
		clk_src = CLK->CLKSEL2 & CLK_CLKSEL2_BPWM1SEL_Msk;
	} else {
		LOG_ERR("Invalid BPWM node");
		return -EINVAL;
	}

	if (clk_src == 0U) {
		/* clock source is from PLL clock */
		bpwm_clk_src = CLK_GetPLLClockFreq();
	} else {
		/* clock source is from PCLK */
		SystemCoreClockUpdate();
		if (bpwm == BPWM0) {
			bpwm_clk_src = CLK_GetPCLK0Freq();
		} else { /* (bpwm == BPWM1) */
			bpwm_clk_src = CLK_GetPCLK1Freq();
		}
	}
	*rate = bpwm_clk_src;
	return 0;
}

static int pwm_numaker_basic_init(const struct device *dev)
{
	const struct pwm_numaker_basic_config *cfg = dev->config;
	struct pwm_numaker_basic_data *data = dev->data;
	BPWM_T *bpwm = cfg->bpwm;
	uint32_t clock_freq;
	int err;

	struct numaker_scc_subsys scc_subsys;

	/* Validate this module's reset object */
	if (!device_is_ready(cfg->reset.dev)) {
		LOG_ERR("reset controller not ready");
		return -ENODEV;
	}

	SYS_UnlockReg();

	/* CLK controller */
	memset(&scc_subsys, 0x00, sizeof(scc_subsys));
	scc_subsys.subsys_id = NUMAKER_SCC_SUBSYS_ID_PCC;
	scc_subsys.pcc.clk_modidx = cfg->clk_modidx;
	scc_subsys.pcc.clk_src = cfg->clk_src;
	scc_subsys.pcc.clk_div = cfg->clk_div;

	/* Equivalent to CLK_EnableModuleClock() */
	err = clock_control_on(cfg->clk_dev, (clock_control_subsys_t)&scc_subsys);
	if (err != 0) {
		goto done;
	}
	/* Equivalent to CLK_SetModuleClock() */
	err = clock_control_configure(cfg->clk_dev, (clock_control_subsys_t)&scc_subsys, NULL);
	if (err != 0) {
		goto done;
	}

	/* Not support standard clock_control_get_rate yet */
	/* clock_control_get_rate(cfg->clk_dev,(clock_control_subsys_t)&scc_subsys,&clock_freq); */
	err =  pwm_numaker_basic_clk_get_rate(bpwm, &clock_freq);

	if (err < 0) {
		LOG_ERR("Get BPWM clock rate failure %d", err);
		goto done;
	}
	data->clock_freq = clock_freq;
	data->cycles_per_sec = data->clock_freq / (cfg->prescale + 1U);

	err = pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_DEFAULT);
	if (err) {
		LOG_ERR("Failed to apply pinctrl state");
		goto done;
	}

	/* Reset PWM to default state, same as BSP's SYS_ResetModule(id_rst) */
	reset_line_toggle_dt(&cfg->reset);

	/* Configure PWM device initially */
	pwm_numaker_basic_configure(dev);

#ifdef CONFIG_PWM_CAPTURE
	/* Enable NVIC */
	cfg->irq_config_func(dev);
#endif

done:
	SYS_LockReg();
	return err;
}

#ifdef CONFIG_PWM_CAPTURE
#define NUMAKER_PWM_IRQ_CONFIG_FUNC(n)                                                             \
	static void pwm_numaker_irq_config_##n(const struct device *dev)                           \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(n, pair0, irq),                                    \
			    DT_INST_IRQ_BY_NAME(n, pair0, priority), pwm_numaker_p0_isr,           \
			    DEVICE_DT_INST_GET(n), 0);                                             \
                                                                                                   \
		irq_enable(DT_INST_IRQ_BY_NAME(n, pair0, irq));                                    \
                                                                                                   \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(n, pair1, irq),                                    \
			    DT_INST_IRQ_BY_NAME(n, pair1, priority), pwm_numaker_p1_isr,           \
			    DEVICE_DT_INST_GET(n), 0);                                             \
                                                                                                   \
		irq_enable(DT_INST_IRQ_BY_NAME(n, pair1, irq));                                    \
                                                                                                   \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(n, pair2, irq),                                    \
			    DT_INST_IRQ_BY_NAME(n, pair2, priority), pwm_numaker_p2_isr,           \
			    DEVICE_DT_INST_GET(n), 0);                                             \
                                                                                                   \
		irq_enable(DT_INST_IRQ_BY_NAME(n, pair2, irq));                                    \
	}
#define IRQ_FUNC_INIT(n) .irq_config_func = pwm_numaker_irq_config_##n
#else
#define NUMAKER_PWM_IRQ_CONFIG_FUNC(n)
#define IRQ_FUNC_INIT(n)
#endif

#define NUMAKER_BPWM_INIT(inst)                                                                     \
	PINCTRL_DT_INST_DEFINE(inst);                                                              \
	NUMAKER_PWM_IRQ_CONFIG_FUNC(inst)                                                          \
                                                                                                   \
	static const struct pwm_numaker_basic_config pwm_numaker_cfg_##inst = {                          \
		.bpwm = (BPWM_T *)DT_INST_REG_ADDR(inst),                                          \
		.prescale = DT_INST_PROP(inst, prescaler),                                         \
		.reset = RESET_DT_SPEC_INST_GET(inst),                                             \
		.clk_modidx = DT_INST_CLOCKS_CELL(inst, clock_module_index),                       \
		.clk_src = DT_INST_CLOCKS_CELL(inst, clock_source),                                \
		.clk_div = DT_INST_CLOCKS_CELL(inst, clock_divider),                               \
		.clk_dev = DEVICE_DT_GET(DT_PARENT(DT_INST_CLOCKS_CTLR(inst))),                    \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                    \
		IRQ_FUNC_INIT(inst)};                                                              \
                                                                                                   \
	static struct pwm_numaker_basic_data pwm_numaker_basic_data_##inst;                                    \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, &pwm_numaker_basic_init, NULL, &pwm_numaker_basic_data_##inst,             \
			      &pwm_numaker_cfg_##inst, PRE_KERNEL_1,                               \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &pwm_numaker_driver_api);

DT_INST_FOREACH_STATUS_OKAY(NUMAKER_BPWM_INIT)
