/*
 * Copyright (c) 2024 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nuvoton_numaker_wdt

#include <zephyr/kernel.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/clock_control_numaker.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/logging/log.h>
#include <soc.h>
#include <NuMicro.h>

LOG_MODULE_REGISTER(wdt_numaker, CONFIG_WDT_LOG_LEVEL);

#define WDT_TOUT_MAX 7U

/* Device config */
struct wdt_numaker_config {
	/* wdt base address */
	WDT_T *wdt_base;
	uint32_t clk_modidx;
	uint32_t clk_src;
	uint32_t clk_div;
	const struct device *clk_dev;
};

struct wdt_numaker_data {
	wdt_callback_t cb;
	bool timeout_valid;
	// Value of the TOUT bits in the WDT->CTL register
	uint8_t tout;
};

static int wdt_numaker_clk_get_rate(const struct wdt_numaker_config *cfg, uint32_t *rate)
{
	if (cfg->clk_src == CLK_CLKSEL1_WDTSEL_LIRC) {
		*rate = __LIRC / (cfg->clk_div + 1);
	} else {
		/* clock source is from HCLK, CLK_CLKSEL1_WDTSEL_HCLK_DIV2048 */
		SystemCoreClockUpdate();
		*rate = CLK_GetHCLKFreq() / 2048 / (cfg->clk_div + 1);
	}

	return 0;
}


/* Convert watchdog clock to nearest ms (rounded up) */
static uint32_t wdt_numaker_calc_ms(const struct device *dev, uint32_t tout)
{
	const struct wdt_numaker_config *cfg = dev->config;
	uint32_t clk_freq;
	uint32_t tout_clks;
	uint32_t period_ms;

	wdt_numaker_clk_get_rate(cfg, &clk_freq);
	tout_clks = (1 << ((2*tout) + 4));
	period_ms = DIV_ROUND_UP(tout_clks * MSEC_PER_SEC, clk_freq);

	return period_ms;
}

static int wdt_numaker_install_timeout(const struct device *dev,
					const struct wdt_timeout_cfg *cfg)
{
	struct wdt_numaker_data *data = dev->data;

	if (cfg->window.min != 0U) {
		LOG_ERR("wdt does not support windowing");
		return -EINVAL;
	}

	// Not valid until we find the request timeout value
	data->timeout_valid = false;

	// Find timeout value
	for (int tout = 0; tout <= WDT_TOUT_MAX; ++tout) {
		if (wdt_numaker_calc_ms(dev, tout) >= cfg->window.max) {
			data->tout = tout;
			data->timeout_valid = true;
			break;
		}
	}

	if( !data->timeout_valid ){
		LOG_ERR("%ums > max: %u", cfg->window.max, 
								  wdt_numaker_calc_ms(dev, WDT_TOUT_MAX));
		return -EINVAL;
	}

	data->cb = cfg->callback;

	return 0;
}

static int wdt_numaker_disable(const struct device *dev)
{
	const struct wdt_numaker_config *config = dev->config;

	SYS_UnlockReg();

	config->wdt_base->CTL &= ~WDT_CTL_WDTEN_Msk;

	SYS_LockReg();

	return 0;
}

static int wdt_numaker_setup(const struct device *dev, uint8_t options)
{
	struct wdt_numaker_data *data = dev->data;
	const struct wdt_numaker_config *cfg = dev->config;
	WDT_T *wdt_base = cfg->wdt_base;
	uint32_t dbg_mask = WDT_CTL_ICEDEBUG_Msk;

	LOG_DBG("");
	irq_disable(DT_INST_IRQN(0));

	if (!data->timeout_valid) {
		LOG_ERR("No valid timeout installed");
		return -EINVAL;
	}

	if (options & WDT_OPT_PAUSE_IN_SLEEP) {
		LOG_ERR("WDT_OPT_PAUSE_IN_SLEEP is not supported");
		return -ENOTSUP;
	}

	if (options & WDT_OPT_PAUSE_HALTED_BY_DBG) {
		dbg_mask = 0U;
	}

	SYS_UnlockReg();
			
	// If the WDT expires, it must be serviced within: 0 = 1026 * WDT_CLK
	// or a system reset is triggered. With a 10kHz clock this is 102.6ms.
	wdt_base->ALTCTL = 0;

	// Open WDT and start counting
	wdt_base->CTL = (data->tout << WDT_CTL_TOUTSEL_Pos) | WDT_CTL_WDTEN_Msk | 
		WDT_CTL_INTEN_Msk | WDT_CTL_RSTCNT_Msk | WDT_CTL_RSTEN_Msk | dbg_mask;

	SYS_LockReg();

	irq_enable(DT_INST_IRQN(0));

	while(wdt_base->CTL & WDT_CTL_SYNC_Msk);

	return 0;
}

static int wdt_numaker_feed(const struct device *dev, int channel_id)
{
	const struct wdt_numaker_config *config = dev->config;

	ARG_UNUSED(channel_id);
	
	/* Reload WDT Counter */
	config->wdt_base->RSTCNT = WDT_RESET_COUNTER_KEYWORD;

	return 0;
}

static void wdt_numaker_isr(const struct device *dev)
{
	struct wdt_numaker_data *data = dev->data;
	const struct wdt_numaker_config *cfg = dev->config;
	WDT_T *wdt_base = cfg->wdt_base;

	if (wdt_base->CTL & WDT_CTL_IF_Msk) {
		/* Clear WDT Timeout Interrupt Flag */
		(wdt_base->CTL = (wdt_base->CTL & ~(WDT_CTL_RSTF_Msk | WDT_CTL_WKF_Msk)) | WDT_CTL_IF_Msk);

		if (data->cb != NULL) {
			data->cb(dev, 0);
		}
	}
}

static DEVICE_API(wdt, wdt_numaker_api) = {
	.setup = wdt_numaker_setup,
	.disable = wdt_numaker_disable,
	.install_timeout = wdt_numaker_install_timeout,
	.feed = wdt_numaker_feed,
};

static int wdt_numaker_init(const struct device *dev)
{
	const struct wdt_numaker_config *cfg = dev->config;
	struct numaker_scc_subsys scc_subsys;
	int err;

	SYS_UnlockReg();

	// Reset WDT on startup
	cfg->wdt_base->CTL = 0;

	irq_disable(DT_INST_IRQN(0));
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

	/* Enable NVIC */
	IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority),
		    wdt_numaker_isr, DEVICE_DT_INST_GET(0), 0);
	irq_enable(DT_INST_IRQN(0));

done:
	SYS_LockReg();
	return err;

}

/* Set config based on DTS */
static struct wdt_numaker_config wdt_numaker_cfg_inst = {
	.wdt_base = (WDT_T *)DT_INST_REG_ADDR(0),
	.clk_modidx = DT_INST_CLOCKS_CELL(0, clock_module_index),
	.clk_src = DT_INST_CLOCKS_CELL(0, clock_source),
	.clk_div = DT_INST_CLOCKS_CELL(0, clock_divider),
	.clk_dev = DEVICE_DT_GET(DT_PARENT(DT_INST_CLOCKS_CTLR(0))),
};

static struct wdt_numaker_data wdt_numaker_data_inst;

DEVICE_DT_INST_DEFINE(0, wdt_numaker_init, NULL,
		      &wdt_numaker_data_inst, &wdt_numaker_cfg_inst,
		      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		      &wdt_numaker_api);
