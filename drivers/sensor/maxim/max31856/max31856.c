/*
 * Copyright (c) 2020 Christian Hirsch
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT maxim_max31856

#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/init.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <zephyr/types.h>
#include <zephyr/device.h>

#define THERMOCOUPLE_TEMPERATURE_POS 18
#define INTERNAL_TEMPERATURE_POS     4
#define THERMOCOUPLE_SIGN_BITS	     0xffff2000
#define INTERNAL_SIGN_BITS	     0xfffff800
#define THERMOCOUPLE_RESOLUTION	     25
#define INTERNAL_RESOLUTION	     625

LOG_MODULE_REGISTER(MAX31856, CONFIG_SENSOR_LOG_LEVEL);

enum max31856_tc_type {
	MAX31856_TC_TYPE_B,
	MAX31856_TC_TYPE_E,
	MAX31856_TC_TYPE_J,
	MAX31856_TC_TYPE_K,
	MAX31856_TC_TYPE_N,
	MAX31856_TC_TYPE_R,
	MAX31856_TC_TYPE_S,
	MAX31856_TC_TYPE_T,
};

enum max31856_samples_avg {
	MAX31856_AVG_1_SAMPLE,
	MAX31856_AVG_2_SAMPLE,
	MAX31856_AVG_4_SAMPLE,
	MAX31856_AVG_8_SAMPLE,
	MAX31856_AVG_16_SAMPLE,
};

struct max31856_cr0 {
	// Conversion Mode
	//
 	// 0 = Normally Off mode (default)
 	// 1 = Automatic Conversion mode. Conversions occur continuously every 100ms (nominal).
	uint8_t cmode: 1;
	// One-Shot Mode
	// 
 	// 0 = No conversions requested (default)
 	// 1 = This causes a single cold-junction and thermocouple conversion to take place when Conversion 
	// Mode bit =0 (normally off mode). The conversion is triggered when CS goes high after writing a 1 to 
	// this bit. Note that if a multi-byte write is performed, the conversion is triggered when CS goes high 
	// at the end of the transaction. A single conversion requires approximately 143ms in 60Hz filter mode 
	// or 169ms in 50Hz filter mode to complete. This bit self clears to 0.
	uint8_t one_shot: 1;
	uint8_t oc_fault: 2;
	// Cold-Junction Sensor Disable
 	//
	// 0 = Cold-junction temperature sensor enabled (default)
	// 1 = Cold-junction temperature sensor disabled. Data from an external temperature sensor may be 
	// written to the cold-junction temperature register. When this bit changes from 0 to 1, the most recent 
	// cold-junction temperature value will remain in the cold-junction temperature register until the internal 
	// sensor is enabled or until a new value is written to the register. The overall temperature conversion 
	// time is reduced by 25ms (typ) when this bit is set to 1.
	uint8_t cj: 1;
	// Fault Mode
	// 
	// 0 = Comparator Mode. The FAULT output and respective fault bit reflects the state of any non
	// masked faults by asserting when the fault condition is true, and deasserting when the fault condition 
	// is no longer true. There is a 2°C hysteresis when in comparator mode for threshold fault conditions. 
	// (default)
	// 1 = Interrupt Mode. The FAULT output and respective fault bit asserts when a non-masked fault 
	// condition is true and remain asserted until a 1 is written to the Fault Status Clear bit. This deasserts 
	// FAULT and respective fault bit until a new fault is detected (note that this may occur immediately if 
	// the fault condition is still in place).
	uint8_t fault: 1;
	// Fault Status Clear
	// 
	// 	0 = Default
	// 	1 = When in interrupt mode, returns all Fault Status bits [7:0] in the Fault Status Register (0Fh) to 0 
	// 	and deasserts the FAULT output. This bit has no effect in comparator mode. Note that the FAULT 
	// 	output and the fault bit may reassert immediately if the fault persists. To prevent the FAULT output 
	// 	from reasserting, first set the Fault Mask bits. The fault status clear bit self-clears to 0.
	uint8_t fault_clr: 1;
	// 50Hz/60Hz Noise Rejection Filter Selection
	//
	// 0= Selects rejection of 60Hz and its harmonics (default)
	// 1= Selects rejection of 50Hz and its harmonics
	// Note: Change the notch frequency only while in the “Normally Off” mode – not in the Automatic 
	// Conversion mode.
	uint8_t filter_50_or_60_hz: 1;
};

struct max31856_cr1 {
	enum max31856_samples_avg avg_sel: 3;
	enum max31856_tc_type tc_type: 4;
};

struct max31856_config {
	struct spi_dt_spec spi;
	uint16_t samples_to_avg;
	enum max31856_tc_type tc_type;
	struct gpio_dt_spec fault_gpio;
	struct gpio_dt_spec drdy_gpio;
};

struct max31856_data {
	// Pointer to base device structure
	const struct device *dev;
	uint32_t sample;
	struct k_work read_data_work;
	struct gpio_callback drdy_gpio_cb;
};

static int max31856_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct max31856_data *data = dev->data;
	const struct max31856_config *config = dev->config;
	int ret;

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL);

	struct spi_buf rx_buf = {
		.buf = &(data->sample),
		.len = sizeof(data->sample),
	};
	const struct spi_buf_set rx = {.buffers = &rx_buf, .count = 1};

	ret = spi_read_dt(&config->spi, &rx);
	if (ret < 0) {
		LOG_ERR("max31856_read FAIL %d", ret);
		return ret;
	}

	return 0;
}

static int max31856_channel_get(const struct device *dev, enum sensor_channel chan,
				struct sensor_value *val)
{
	struct max31856_data *data = dev->data;
	uint32_t temp = sys_be32_to_cpu(data->sample);

	if (temp & BIT(16)) {
		return -EIO;
	}

	switch (chan) {

	case SENSOR_CHAN_AMBIENT_TEMP:
		temp = (temp >> THERMOCOUPLE_TEMPERATURE_POS) & 0x3fff;

		/* if sign bit is set, make value negative */
		if (temp & BIT(14)) {
			temp |= THERMOCOUPLE_SIGN_BITS;
		}

		temp = temp * THERMOCOUPLE_RESOLUTION;

		val->val1 = temp / 100;
		val->val2 = (temp - val->val1 * 100) * 10000;
		break;

	case SENSOR_CHAN_DIE_TEMP:
		temp = (temp >> INTERNAL_TEMPERATURE_POS) & 0xfff;

		/* if sign bit is set, make value negative */
		if (temp & BIT(12)) {
			temp |= INTERNAL_SIGN_BITS;
		}

		temp = temp * INTERNAL_RESOLUTION;

		val->val1 = temp / 10000;
		val->val2 = (temp - val->val1 * 10000) * 100;
		break;

	default:
		return -ENOTSUP;
	}

	return 0;
}

int max31856_trigger_set(const struct device *dev, const struct sensor_trigger *trig,
				    	 sensor_trigger_handler_t handler){
	
	struct max31856_config *cfg = dev->config;

	switch(trig->type){
		case SENSOR_TRIG_TIMER:			// Fall-through
		case SENSOR_TRIG_DATA_READY:
			// Disable trigger
			if( handler == NULL ){
				switch(trig->chan){
					case SENSOR_CHAN_ALL:			// Fall-through
					case SENSOR_CHAN_AMBIENT_TEMP:
						GPIO_DisableInt(cfg->drdy_gpio.port, cfg->drdy_gpio.pin);
						break;
					default:
						return -ENOTSUP;
				}
				return 0;
			}
			int i;

			switch(trig->chan){
				case SENSOR_CHAN_ALL:
				case SENSOR_CHAN_AMBIENT_TEMP:
					GPIO_EnableInt(cfg->drdy_gpio.port, cfg->drdy_gpio.pin, GPIO_INT_FALLING);
					return 0;
				default:
					return -ENOTSUP;
			}

		default:
			break;
	}
	return -ENOTSUP;
}

/**
 * @brief Work handler to read temperature data. This is in a work handler
 * so the trigger handler can be called from a non-ISR context if it is enabled.
 */
static void max31856_read_work_handler(struct k_work *work){
	
	// Read temperature data

}

/**
 * @brief GPIO callback for the data ready GPIO. This just submits work to read data
 * from the MAX31856 temperature sensor.
 */
static void max31856_drdy_handler(const struct device *port, struct gpio_callback *cb,
								  gpio_port_pins_t pins){
	int ret;
	struct max31856_data *data = CONTAINER_OF(cb, struct max31856_data, drdy_gpio_cb);

	ret = k_work_submit(&data->read_data_work);
	if( ret != 0 ){
		LOG_ERR("Failed to submit work data.");
	}
}

static DEVICE_API(sensor, max31856_api) = {
	.sample_fetch = max31856_sample_fetch,
	.channel_get  = max31856_channel_get,
	.trigger_set  = max31856_trigger_set,
};

static int max31856_init(const struct device *dev)
{
	const struct max31856_config *config = dev->config;
	struct max31856_data *data = dev->data;

	if (!spi_is_ready_dt(&config->spi)) {
		LOG_ERR("SPI bus is not ready");
		return -ENODEV;
	}

	if(!gpio_is_ready_dt(&config->drdy_gpio)){
		LOG_ERR("drdy pin not ready");
		return -ENODEV;
	}

	if(!gpio_is_ready_dt(&config->fault_gpio)){
		LOG_ERR("fault pin not ready");
		return -ENODEV;
	}

	// Setup pins
	gpio_pin_configure_dt(&config->drdy_gpio, GPIO_ACTIVE_LOW);
	gpio_pin_configure_dt(&config->fault_gpio, GPIO_ACTIVE_LOW);

	// Set callback for data ready pin
	gpio_init_callback(&data->drdy_gpio_cb, max31856_drdy_handler, 
					   BIT(config->fault_gpio.pin));

	return 0;
}

#define MAX31856_INIT(n)                                                                    		\
	static struct max31856_data max31856_data_##n = {												\
		.read_data_work = Z_WORK_INITIALIZER(max31856_read_work_handler)
	};                                          		\
	static const struct max31856_config max31856_config_##n = {                             		\
		.spi 			= SPI_DT_SPEC_INST_GET(n, SPI_OP_MODE_MASTER | SPI_WORD_SET(8U), 0U),   	\
		.samples_to_avg = CONCAT(MAX31856_AVG_, DT_INST_PROP(n, num_sample_to_avg), _SAMPLE),		\
		.tc_type 		= CONCAT(MAX31856_TC_TYPE_, DT_INST_STRING_UNQUOTED(n, thermocouple_type)),	\
		.fault_gpio		= GPIO_DT_SPEC_INST_GET(n, fault_gpios),									\
		.drdy_gpio		= GPIO_DT_SPEC_INST_GET(n, drdy_gpios),										\
	};                                                                                      		\
	SENSOR_DEVICE_DT_INST_DEFINE(n, &max31856_init, NULL, &max31856_data_##n,                  		\
				     &max31856_config_##n, POST_KERNEL,                            					\
				     CONFIG_SENSOR_INIT_PRIORITY, &max31856_api);

DT_INST_FOREACH_STATUS_OKAY(MAX31856_INIT)
