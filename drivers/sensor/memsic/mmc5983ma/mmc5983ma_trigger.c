/*
 * Copyright (c) 2026 Anastasios Kaltakis
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * MMC5983MA data-ready trigger on the INT pin.
 *
 * INT is an active-high level output: it rises when a measurement
 * (magnetic or temperature) completes and stays high until the done bit is
 * cleared in the status register. The driver clears it after the handler
 * ran, and sample_fetch() clears it as well.
 */

#define DT_DRV_COMPAT memsic_mmc5983ma

#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "mmc5983ma.h"

LOG_MODULE_DECLARE(MMC5983MA, CONFIG_SENSOR_LOG_LEVEL);

static void mmc5983ma_handle_int(const struct device *dev)
{
	struct mmc5983ma_data *data = dev->data;

	if (data->handler != NULL) {
		data->handler(dev, data->trigger);
	}

	/* If the handler did not fetch, release the INT line anyway. */
	(void)mmc5983ma_reg_write(dev, MMC5983MA_REG_STATUS,
				  MMC5983MA_STATUS_MEAS_M_DONE | MMC5983MA_STATUS_MEAS_T_DONE);
}

static void mmc5983ma_gpio_callback(const struct device *port, struct gpio_callback *cb,
				    uint32_t pins)
{
	struct mmc5983ma_data *data = CONTAINER_OF(cb, struct mmc5983ma_data, gpio_cb);

	ARG_UNUSED(port);
	ARG_UNUSED(pins);

#if defined(CONFIG_MMC5983MA_TRIGGER_OWN_THREAD)
	k_sem_give(&data->gpio_sem);
#elif defined(CONFIG_MMC5983MA_TRIGGER_GLOBAL_THREAD)
	k_work_submit(&data->work);
#endif
}

#if defined(CONFIG_MMC5983MA_TRIGGER_OWN_THREAD)
static void mmc5983ma_thread(void *p1, void *p2, void *p3)
{
	struct mmc5983ma_data *data = p1;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	for (;;) {
		k_sem_take(&data->gpio_sem, K_FOREVER);
		mmc5983ma_handle_int(data->dev);
	}
}
#elif defined(CONFIG_MMC5983MA_TRIGGER_GLOBAL_THREAD)
static void mmc5983ma_work_cb(struct k_work *work)
{
	struct mmc5983ma_data *data = CONTAINER_OF(work, struct mmc5983ma_data, work);

	mmc5983ma_handle_int(data->dev);
}
#endif

int mmc5983ma_trigger_set(const struct device *dev, const struct sensor_trigger *trig,
			  sensor_trigger_handler_t handler)
{
	const struct mmc5983ma_config *cfg = dev->config;
	struct mmc5983ma_data *data = dev->data;
	uint8_t ctrl0;
	int ret;

	if (cfg->int_gpio.port == NULL) {
		return -ENOTSUP;
	}

	if (trig->type != SENSOR_TRIG_DATA_READY ||
	    (trig->chan != SENSOR_CHAN_MAGN_XYZ && trig->chan != SENSOR_CHAN_ALL)) {
		return -ENOTSUP;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	ret = gpio_pin_interrupt_configure_dt(&cfg->int_gpio, GPIO_INT_DISABLE);
	if (ret < 0) {
		goto out;
	}

	data->handler = handler;
	data->trigger = trig;

	ctrl0 = data->ctrl0 & (uint8_t)~MMC5983MA_CTRL0_INT_MEAS_DONE;
	if (handler != NULL) {
		ctrl0 |= MMC5983MA_CTRL0_INT_MEAS_DONE;
	}
	ret = mmc5983ma_reg_write(dev, MMC5983MA_REG_CTRL0, ctrl0);
	if (ret < 0) {
		goto out;
	}
	data->ctrl0 = ctrl0;

	if (handler != NULL) {
		/* Drop a stale flag so the next edge is a real measurement. */
		(void)mmc5983ma_reg_write(dev, MMC5983MA_REG_STATUS,
					  MMC5983MA_STATUS_MEAS_M_DONE |
						  MMC5983MA_STATUS_MEAS_T_DONE);
		ret = gpio_pin_interrupt_configure_dt(&cfg->int_gpio, GPIO_INT_EDGE_TO_ACTIVE);
	}

out:
	k_mutex_unlock(&data->lock);

	return ret;
}

int mmc5983ma_trigger_init(const struct device *dev)
{
	const struct mmc5983ma_config *cfg = dev->config;
	struct mmc5983ma_data *data = dev->data;
	int ret;

	data->dev = dev;

	if (cfg->int_gpio.port == NULL) {
		LOG_DBG("no INT line, triggers unavailable");
		return 0;
	}

	if (!gpio_is_ready_dt(&cfg->int_gpio)) {
		LOG_ERR("INT GPIO not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&cfg->int_gpio, GPIO_INPUT);
	if (ret < 0) {
		return ret;
	}

	gpio_init_callback(&data->gpio_cb, mmc5983ma_gpio_callback, BIT(cfg->int_gpio.pin));
	ret = gpio_add_callback(cfg->int_gpio.port, &data->gpio_cb);
	if (ret < 0) {
		return ret;
	}

#if defined(CONFIG_MMC5983MA_TRIGGER_OWN_THREAD)
	k_sem_init(&data->gpio_sem, 0, K_SEM_MAX_LIMIT);
	k_thread_create(&data->thread, data->thread_stack, CONFIG_MMC5983MA_THREAD_STACK_SIZE,
			mmc5983ma_thread, data, NULL, NULL,
			K_PRIO_COOP(CONFIG_MMC5983MA_THREAD_PRIORITY), 0, K_NO_WAIT);
	k_thread_name_set(&data->thread, "mmc5983ma");
#elif defined(CONFIG_MMC5983MA_TRIGGER_GLOBAL_THREAD)
	k_work_init(&data->work, mmc5983ma_work_cb);
#endif

	return 0;
}
