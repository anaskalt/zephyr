/*
 * Copyright (c) 2026 Anastasios Kaltakis
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * MEMSIC MMC5983MA 3-axis AMR magnetometer, I2C.
 *
 * The four internal control registers are write only, so the driver keeps
 * a shadow of each and always rewrites the whole byte. One-shot bits
 * (TM_M, TM_T, SET, RESET) self-clear and are never kept in the shadow.
 */

#define DT_DRV_COMPAT memsic_mmc5983ma

#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/mmc5983ma.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include "mmc5983ma.h"

LOG_MODULE_REGISTER(MMC5983MA, CONFIG_SENSOR_LOG_LEVEL);

/* Measurement time per bandwidth setting, rounded up to whole ms. */
static const uint8_t mmc5983ma_meas_ms[4] = {8, 4, 2, 1};

/* Continuous mode rates indexed by the CM_Freq code. */
static const uint16_t mmc5983ma_cm_freq_hz[8] = {0, 1, 10, 20, 50, 100, 200, 1000};

int mmc5983ma_reg_write(const struct device *dev, uint8_t reg, uint8_t val)
{
	const struct mmc5983ma_config *cfg = dev->config;

	return i2c_reg_write_byte_dt(&cfg->i2c, reg, val);
}

int mmc5983ma_reg_read(const struct device *dev, uint8_t reg, uint8_t *val)
{
	const struct mmc5983ma_config *cfg = dev->config;

	return i2c_reg_read_byte_dt(&cfg->i2c, reg, val);
}

int mmc5983ma_write_ctrl0(const struct device *dev, uint8_t extra_bits)
{
	struct mmc5983ma_data *data = dev->data;

	return mmc5983ma_reg_write(dev, MMC5983MA_REG_CTRL0, data->ctrl0 | extra_bits);
}

static int mmc5983ma_wait_status(const struct device *dev, uint8_t bit, uint32_t first_wait_ms)
{
	int64_t deadline = k_uptime_get() + MMC5983MA_MEAS_TIMEOUT_MS;
	uint8_t status;
	int ret;

	if (first_wait_ms > 0U) {
		k_sleep(K_MSEC(first_wait_ms));
	}

	for (;;) {
		ret = mmc5983ma_reg_read(dev, MMC5983MA_REG_STATUS, &status);
		if (ret < 0) {
			return ret;
		}
		if ((status & bit) != 0U) {
			return 0;
		}
		if (k_uptime_get() >= deadline) {
			return -ETIMEDOUT;
		}
		k_sleep(K_MSEC(1));
	}
}

static int mmc5983ma_pulse(const struct device *dev, uint8_t bit)
{
	int ret;

	ret = mmc5983ma_write_ctrl0(dev, bit);
	if (ret < 0) {
		return ret;
	}

	/* The coil pulse lasts 500 ns; give the bridge time to settle. */
	k_busy_wait(MMC5983MA_SET_RESET_TIME_US);

	return 0;
}

static int mmc5983ma_fetch_magn(const struct device *dev)
{
	const struct mmc5983ma_config *cfg = dev->config;
	struct mmc5983ma_data *data = dev->data;
	uint8_t buf[7];
	uint32_t first_wait = 0;
	int ret;

	if (data->odr == 0U) {
		ret = mmc5983ma_write_ctrl0(dev, MMC5983MA_CTRL0_TM_M);
		if (ret < 0) {
			return ret;
		}
		first_wait = mmc5983ma_meas_ms[data->ctrl1 & MMC5983MA_CTRL1_BW_MASK];
		if (data->ctrl0 & MMC5983MA_CTRL0_AUTO_SR_EN) {
			/* SET and RESET pulses precede the measurement. */
			first_wait += 1U;
		}
	}

	ret = mmc5983ma_wait_status(dev, MMC5983MA_STATUS_MEAS_M_DONE, first_wait);
	if (ret < 0) {
		return ret;
	}

	ret = i2c_burst_read_dt(&cfg->i2c, MMC5983MA_REG_XOUT0, buf, sizeof(buf));
	if (ret < 0) {
		return ret;
	}

	data->raw[0] = ((uint32_t)buf[0] << 10) | ((uint32_t)buf[1] << 2) | ((buf[6] >> 6) & 0x3U);
	data->raw[1] = ((uint32_t)buf[2] << 10) | ((uint32_t)buf[3] << 2) | ((buf[6] >> 4) & 0x3U);
	data->raw[2] = ((uint32_t)buf[4] << 10) | ((uint32_t)buf[5] << 2) | ((buf[6] >> 2) & 0x3U);

	/* Clear the done flag (and the interrupt) for the next measurement. */
	return mmc5983ma_reg_write(dev, MMC5983MA_REG_STATUS, MMC5983MA_STATUS_MEAS_M_DONE);
}

static int mmc5983ma_fetch_temp(const struct device *dev)
{
	struct mmc5983ma_data *data = dev->data;
	int ret;

	if (data->odr != 0U) {
		/* No temperature measurement while the continuous mode runs. */
		return -ENOTSUP;
	}

	ret = mmc5983ma_write_ctrl0(dev, MMC5983MA_CTRL0_TM_T);
	if (ret < 0) {
		return ret;
	}

	ret = mmc5983ma_wait_status(dev, MMC5983MA_STATUS_MEAS_T_DONE, 1);
	if (ret < 0) {
		return ret;
	}

	ret = mmc5983ma_reg_read(dev, MMC5983MA_REG_TOUT, &data->raw_temp);
	if (ret < 0) {
		return ret;
	}
	data->temp_valid = true;

	return mmc5983ma_reg_write(dev, MMC5983MA_REG_STATUS, MMC5983MA_STATUS_MEAS_T_DONE);
}

static int mmc5983ma_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct mmc5983ma_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);

	switch (chan) {
	case SENSOR_CHAN_MAGN_X:
	case SENSOR_CHAN_MAGN_Y:
	case SENSOR_CHAN_MAGN_Z:
	case SENSOR_CHAN_MAGN_XYZ:
		ret = mmc5983ma_fetch_magn(dev);
		break;
	case SENSOR_CHAN_DIE_TEMP:
		ret = mmc5983ma_fetch_temp(dev);
		break;
	case SENSOR_CHAN_ALL:
		ret = mmc5983ma_fetch_magn(dev);
		if (ret == 0 && data->odr == 0U) {
			ret = mmc5983ma_fetch_temp(dev);
		}
		break;
	default:
		ret = -ENOTSUP;
		break;
	}

	k_mutex_unlock(&data->lock);

	return ret;
}

static void mmc5983ma_magn_to_value(uint32_t raw, struct sensor_value *val)
{
	int64_t delta = (int64_t)raw - MMC5983MA_NULL_FIELD_COUNTS;
	int64_t micro_gauss = (delta * 1000000LL) / MMC5983MA_COUNTS_PER_GAUSS;

	(void)sensor_value_from_micro(val, micro_gauss);
}

static int mmc5983ma_channel_get(const struct device *dev, enum sensor_channel chan,
				 struct sensor_value *val)
{
	struct mmc5983ma_data *data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_MAGN_X:
		mmc5983ma_magn_to_value(data->raw[0], val);
		break;
	case SENSOR_CHAN_MAGN_Y:
		mmc5983ma_magn_to_value(data->raw[1], val);
		break;
	case SENSOR_CHAN_MAGN_Z:
		mmc5983ma_magn_to_value(data->raw[2], val);
		break;
	case SENSOR_CHAN_MAGN_XYZ:
		mmc5983ma_magn_to_value(data->raw[0], &val[0]);
		mmc5983ma_magn_to_value(data->raw[1], &val[1]);
		mmc5983ma_magn_to_value(data->raw[2], &val[2]);
		break;
	case SENSOR_CHAN_DIE_TEMP:
		if (!data->temp_valid) {
			return -ENODATA;
		}
		/* -75 C at code 0, 0.8 C per LSB. */
		(void)sensor_value_from_milli(val, (int64_t)data->raw_temp * 800 - 75000);
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int mmc5983ma_set_odr(const struct device *dev, uint16_t hz)
{
	struct mmc5983ma_data *data = dev->data;
	uint8_t bw = data->ctrl1 & MMC5983MA_CTRL1_BW_MASK;
	uint8_t code = 0xFF;
	uint8_t ctrl2;
	int ret;

	for (uint8_t i = 0; i < ARRAY_SIZE(mmc5983ma_cm_freq_hz); i++) {
		if (mmc5983ma_cm_freq_hz[i] == hz) {
			code = i;
			break;
		}
	}
	if (code == 0xFF) {
		return -EINVAL;
	}
	if ((hz == 200U && bw < 1U) || (hz == 1000U && bw != 3U)) {
		return -EINVAL;
	}

	ctrl2 = data->ctrl2 & (uint8_t)~(MMC5983MA_CTRL2_CM_FREQ_MASK | MMC5983MA_CTRL2_CMM_EN);
	if (code != 0U) {
		ctrl2 |= code | MMC5983MA_CTRL2_CMM_EN;
	}

	ret = mmc5983ma_reg_write(dev, MMC5983MA_REG_CTRL2, ctrl2);
	if (ret < 0) {
		return ret;
	}

	data->ctrl2 = ctrl2;
	data->odr = hz;

	return 0;
}

static int mmc5983ma_attr_set(const struct device *dev, enum sensor_channel chan,
			      enum sensor_attribute attr, const struct sensor_value *val)
{
	struct mmc5983ma_data *data = dev->data;
	uint8_t reg;
	int ret;

	if (chan != SENSOR_CHAN_MAGN_XYZ && chan != SENSOR_CHAN_ALL) {
		return -ENOTSUP;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	switch ((int)attr) {
	case SENSOR_ATTR_SAMPLING_FREQUENCY:
		if (val->val1 < 0 || val->val1 > 1000) {
			ret = -EINVAL;
			break;
		}
		ret = mmc5983ma_set_odr(dev, (uint16_t)val->val1);
		break;
	case MMC5983MA_ATTR_BANDWIDTH:
		if (val->val1 < 0 || val->val1 > 3) {
			ret = -EINVAL;
			break;
		}
		if ((data->odr == 200U && val->val1 < 1) || (data->odr == 1000U && val->val1 != 3)) {
			ret = -EINVAL;
			break;
		}
		reg = (data->ctrl1 & (uint8_t)~MMC5983MA_CTRL1_BW_MASK) | (uint8_t)val->val1;
		ret = mmc5983ma_reg_write(dev, MMC5983MA_REG_CTRL1, reg);
		if (ret == 0) {
			data->ctrl1 = reg;
		}
		break;
	case MMC5983MA_ATTR_AUTO_SET_RESET:
		reg = data->ctrl0 & (uint8_t)~MMC5983MA_CTRL0_AUTO_SR_EN;
		if (val->val1 != 0) {
			reg |= MMC5983MA_CTRL0_AUTO_SR_EN;
		}
		ret = mmc5983ma_reg_write(dev, MMC5983MA_REG_CTRL0, reg);
		if (ret == 0) {
			data->ctrl0 = reg;
		}
		break;
	case MMC5983MA_ATTR_SET:
		ret = mmc5983ma_pulse(dev, MMC5983MA_CTRL0_SET);
		break;
	case MMC5983MA_ATTR_RESET:
		ret = mmc5983ma_pulse(dev, MMC5983MA_CTRL0_RESET);
		break;
	case MMC5983MA_ATTR_PERIODIC_SET:
		if (val->val1 < 0 || val->val1 > 7) {
			ret = -EINVAL;
			break;
		}
		reg = data->ctrl2 &
		      (uint8_t)~(MMC5983MA_CTRL2_PRD_SET_MASK | MMC5983MA_CTRL2_EN_PRD_SET);
		reg |= (uint8_t)(val->val1 << 4);
		if (val->val2 != 0) {
			reg |= MMC5983MA_CTRL2_EN_PRD_SET;
		}
		ret = mmc5983ma_reg_write(dev, MMC5983MA_REG_CTRL2, reg);
		if (ret == 0) {
			data->ctrl2 = reg;
		}
		break;
	default:
		ret = -ENOTSUP;
		break;
	}

	k_mutex_unlock(&data->lock);

	return ret;
}

static int mmc5983ma_attr_get(const struct device *dev, enum sensor_channel chan,
			      enum sensor_attribute attr, struct sensor_value *val)
{
	struct mmc5983ma_data *data = dev->data;

	if (chan != SENSOR_CHAN_MAGN_XYZ && chan != SENSOR_CHAN_ALL) {
		return -ENOTSUP;
	}

	val->val2 = 0;

	switch ((int)attr) {
	case SENSOR_ATTR_SAMPLING_FREQUENCY:
		val->val1 = data->odr;
		break;
	case MMC5983MA_ATTR_BANDWIDTH:
		val->val1 = data->ctrl1 & MMC5983MA_CTRL1_BW_MASK;
		break;
	case MMC5983MA_ATTR_AUTO_SET_RESET:
		val->val1 = (data->ctrl0 & MMC5983MA_CTRL0_AUTO_SR_EN) ? 1 : 0;
		break;
	case MMC5983MA_ATTR_PERIODIC_SET:
		val->val1 = (data->ctrl2 & MMC5983MA_CTRL2_PRD_SET_MASK) >> 4;
		val->val2 = (data->ctrl2 & MMC5983MA_CTRL2_EN_PRD_SET) ? 1 : 0;
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int mmc5983ma_init(const struct device *dev)
{
	const struct mmc5983ma_config *cfg = dev->config;
	struct mmc5983ma_data *data = dev->data;
	uint8_t id = 0;
	uint8_t status = 0;
	int ret;

	k_mutex_init(&data->lock);

	if (!i2c_is_ready_dt(&cfg->i2c)) {
		LOG_ERR("I2C bus not ready");
		return -ENODEV;
	}

	ret = mmc5983ma_reg_read(dev, MMC5983MA_REG_PRODUCT_ID, &id);
	if (ret < 0) {
		LOG_ERR("product ID read failed (%d)", ret);
		return ret;
	}
	if (id != MMC5983MA_PRODUCT_ID) {
		LOG_ERR("unexpected product ID 0x%02x", id);
		return -ENODEV;
	}

	/* Software reset: clears every register and re-reads the OTP. */
	ret = mmc5983ma_reg_write(dev, MMC5983MA_REG_CTRL1, MMC5983MA_CTRL1_SW_RST);
	if (ret < 0) {
		return ret;
	}
	k_sleep(K_MSEC(MMC5983MA_POR_TIME_MS));

	ret = mmc5983ma_reg_read(dev, MMC5983MA_REG_STATUS, &status);
	if (ret < 0) {
		return ret;
	}
	if ((status & MMC5983MA_STATUS_OTP_RD_DONE) == 0U) {
		LOG_WRN("OTP not read after reset (status 0x%02x)", status);
	}

	data->ctrl1 = cfg->bandwidth & MMC5983MA_CTRL1_BW_MASK;
	ret = mmc5983ma_reg_write(dev, MMC5983MA_REG_CTRL1, data->ctrl1);
	if (ret < 0) {
		return ret;
	}

	data->ctrl0 = cfg->auto_set_reset ? MMC5983MA_CTRL0_AUTO_SR_EN : 0U;
	ret = mmc5983ma_reg_write(dev, MMC5983MA_REG_CTRL0, data->ctrl0);
	if (ret < 0) {
		return ret;
	}

	data->ctrl2 = 0U;
	data->odr = 0U;
	data->temp_valid = false;

	/* Condition the sensing elements once before the first measurement. */
	ret = mmc5983ma_pulse(dev, MMC5983MA_CTRL0_SET);
	if (ret < 0) {
		return ret;
	}

#ifdef CONFIG_MMC5983MA_TRIGGER
	ret = mmc5983ma_trigger_init(dev);
	if (ret < 0) {
		LOG_ERR("trigger init failed (%d)", ret);
		return ret;
	}
#endif

	LOG_INF("MMC5983MA ready, BW=%u auto-SR=%u", data->ctrl1, cfg->auto_set_reset);

	return 0;
}

static DEVICE_API(sensor, mmc5983ma_driver_api) = {
	.attr_set = mmc5983ma_attr_set,
	.attr_get = mmc5983ma_attr_get,
#ifdef CONFIG_MMC5983MA_TRIGGER
	.trigger_set = mmc5983ma_trigger_set,
#endif
	.sample_fetch = mmc5983ma_sample_fetch,
	.channel_get = mmc5983ma_channel_get,
};

#ifdef CONFIG_MMC5983MA_TRIGGER
#define MMC5983MA_INT_GPIO(inst) .int_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, int_gpios, {0}),
#else
#define MMC5983MA_INT_GPIO(inst)
#endif

#define MMC5983MA_DEFINE(inst)                                                                     \
	static struct mmc5983ma_data mmc5983ma_data_##inst;                                        \
	static const struct mmc5983ma_config mmc5983ma_config_##inst = {                           \
		.i2c = I2C_DT_SPEC_INST_GET(inst),                                                 \
		MMC5983MA_INT_GPIO(inst)                                                           \
		.bandwidth = DT_INST_PROP(inst, bandwidth),                                        \
		.auto_set_reset = DT_INST_PROP(inst, auto_set_reset),                              \
	};                                                                                         \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, mmc5983ma_init, NULL, &mmc5983ma_data_##inst,           \
				     &mmc5983ma_config_##inst, POST_KERNEL,                        \
				     CONFIG_SENSOR_INIT_PRIORITY, &mmc5983ma_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MMC5983MA_DEFINE)
