/*
 * Copyright (c) 2024 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nuvoton_numaker_dac

#include <zephyr/kernel.h>
#include <zephyr/drivers/dac.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/clock_control_numaker.h>
#include <zephyr/logging/log.h>
#include <soc.h>
#include <NuMicro.h>

LOG_MODULE_REGISTER(dac_numaker, CONFIG_DAC_LOG_LEVEL);

struct dac_numaker_config {
	DAC_T *base;
	const struct reset_dt_spec reset;
	uint32_t clk_modidx;
	uint32_t clk_src;
	uint32_t clk_div;
	const struct device *clk_dev;
	const struct pinctrl_dev_config *pincfg;
};

struct dac_numaker_data {
	bool configured;
	uint8_t resolution;
};

static int dac_numaker_channel_setup(const struct device *dev,
				     const struct dac_channel_cfg *channel_cfg)
{
	const struct dac_numaker_config *cfg = dev->config;
	struct dac_numaker_data *data = dev->data;
	DAC_T *dac = cfg->base;

	if (channel_cfg->channel_id != 0) {
		LOG_ERR("Unsupported channel %d", channel_cfg->channel_id);
		return -ENOTSUP;
	}

	if (channel_cfg->resolution != 12 && channel_cfg->resolution != 8) {
		LOG_ERR("Unsupported resolution %d", channel_cfg->resolution);
		return -ENOTSUP;
	}

	SYS_UnlockReg();

	/* Open DAC with software trigger */
	DAC_Open(dac, 0, DAC_SOFTWARE_TRIGGER);

	/* Set resolution: 8-bit or 12-bit */
	if (channel_cfg->resolution == 8) {
		dac->CTL |= DAC_CTL_BWSEL_Msk;
	} else {
		dac->CTL &= ~DAC_CTL_BWSEL_Msk;
	}

	/* Set settling time to 8 microseconds */
	DAC_SetDelayTime(dac, 8);

	if (channel_cfg->buffered) {
		DAC_DISABLE_BYPASS_BUFFER(dac);
	} else {
		DAC_ENABLE_BYPASS_BUFFER(dac);
	}

	SYS_LockReg();

	data->resolution = channel_cfg->resolution;
	data->configured = true;

	return 0;
}

static int dac_numaker_write_value(const struct device *dev, uint8_t channel,
				   uint32_t value)
{
	const struct dac_numaker_config *cfg = dev->config;
	struct dac_numaker_data *data = dev->data;
	DAC_T *dac = cfg->base;

	if (channel != 0) {
		return -ENOTSUP;
	}

	if (!data->configured) {
		return -EINVAL;
	}

	DAC_CLR_INT_FLAG(dac, 0);
	DAC_WRITE_DATA(dac, 0, value);
	DAC_START_CONV(dac);

	return 0;
}

static int dac_numaker_init(const struct device *dev)
{
	const struct dac_numaker_config *cfg = dev->config;
	struct numaker_scc_subsys scc_subsys;
	int err;

	SYS_UnlockReg();

	/* Configure module clock */
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

	/* Apply pin configuration */
	err = pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_DEFAULT);
	if (err != 0) {
		LOG_ERR("Failed to apply pinctrl state");
		goto done;
	}

	/* Reset DAC peripheral */
	reset_line_toggle_dt(&cfg->reset);

done:
	SYS_LockReg();
	return err;
}

static DEVICE_API(dac, dac_numaker_driver_api) = {
	.channel_setup = dac_numaker_channel_setup,
	.write_value = dac_numaker_write_value,
};

#define DAC_NUMAKER_INIT(inst)							\
	PINCTRL_DT_INST_DEFINE(inst);						\
										\
	static const struct dac_numaker_config dac_numaker_cfg_##inst = {	\
		.base = (DAC_T *)DT_INST_REG_ADDR(inst),			\
		.reset = RESET_DT_SPEC_INST_GET(inst),				\
		.clk_modidx =							\
			DT_INST_CLOCKS_CELL(inst, clock_module_index),		\
		.clk_src = DT_INST_CLOCKS_CELL(inst, clock_source),		\
		.clk_div = DT_INST_CLOCKS_CELL(inst, clock_divider),		\
		.clk_dev =							\
			DEVICE_DT_GET(DT_PARENT(DT_INST_CLOCKS_CTLR(inst))),	\
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),		\
	};									\
										\
	static struct dac_numaker_data dac_numaker_data_##inst;			\
										\
	DEVICE_DT_INST_DEFINE(inst, dac_numaker_init, NULL,			\
			      &dac_numaker_data_##inst,				\
			      &dac_numaker_cfg_##inst,				\
			      POST_KERNEL, CONFIG_DAC_INIT_PRIORITY,		\
			      &dac_numaker_driver_api);

DT_INST_FOREACH_STATUS_OKAY(DAC_NUMAKER_INIT)
