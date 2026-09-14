/*
 * Copyright (c) 2026 Anastasios Kaltakis
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_MEMSIC_MMC5983MA_MMC5983MA_H_
#define ZEPHYR_DRIVERS_SENSOR_MEMSIC_MMC5983MA_MMC5983MA_H_

#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/mmc5983ma.h>
#include <zephyr/kernel.h>

/* Register map (datasheet rev. A, "REGISTER MAP"). */
#define MMC5983MA_REG_XOUT0     0x00
#define MMC5983MA_REG_XOUT1     0x01
#define MMC5983MA_REG_YOUT0     0x02
#define MMC5983MA_REG_YOUT1     0x03
#define MMC5983MA_REG_ZOUT0     0x04
#define MMC5983MA_REG_ZOUT1     0x05
#define MMC5983MA_REG_XYZOUT2   0x06
#define MMC5983MA_REG_TOUT      0x07
#define MMC5983MA_REG_STATUS    0x08
#define MMC5983MA_REG_CTRL0     0x09
#define MMC5983MA_REG_CTRL1     0x0A
#define MMC5983MA_REG_CTRL2     0x0B
#define MMC5983MA_REG_CTRL3     0x0C
#define MMC5983MA_REG_PRODUCT_ID 0x2F

#define MMC5983MA_PRODUCT_ID    0x30

/* Status. Writing 1 to the done bits clears the interrupt. */
#define MMC5983MA_STATUS_MEAS_M_DONE BIT(0)
#define MMC5983MA_STATUS_MEAS_T_DONE BIT(1)
#define MMC5983MA_STATUS_OTP_RD_DONE BIT(4)

/* Internal control 0 (write only). */
#define MMC5983MA_CTRL0_TM_M          BIT(0)
#define MMC5983MA_CTRL0_TM_T          BIT(1)
#define MMC5983MA_CTRL0_INT_MEAS_DONE BIT(2)
#define MMC5983MA_CTRL0_SET           BIT(3)
#define MMC5983MA_CTRL0_RESET         BIT(4)
#define MMC5983MA_CTRL0_AUTO_SR_EN    BIT(5)
#define MMC5983MA_CTRL0_OTP_READ      BIT(6)

/* Internal control 1 (write only). */
#define MMC5983MA_CTRL1_BW_MASK       GENMASK(1, 0)
#define MMC5983MA_CTRL1_X_INHIBIT     BIT(2)
#define MMC5983MA_CTRL1_YZ_INHIBIT    GENMASK(4, 3)
#define MMC5983MA_CTRL1_SW_RST        BIT(7)

/* Internal control 2 (write only). */
#define MMC5983MA_CTRL2_CM_FREQ_MASK  GENMASK(2, 0)
#define MMC5983MA_CTRL2_CMM_EN        BIT(3)
#define MMC5983MA_CTRL2_PRD_SET_MASK  GENMASK(6, 4)
#define MMC5983MA_CTRL2_EN_PRD_SET    BIT(7)

/* Internal control 3 (write only). */
#define MMC5983MA_CTRL3_ST_ENP        BIT(1)
#define MMC5983MA_CTRL3_ST_ENM        BIT(2)
#define MMC5983MA_CTRL3_SPI_3W        BIT(6)

/* Timing. */
#define MMC5983MA_POR_TIME_MS         15   /* "power on time is 10 ms" */
#define MMC5983MA_SET_RESET_TIME_US   100  /* pulse is 500 ns */
#define MMC5983MA_MEAS_TIMEOUT_MS     30

struct mmc5983ma_config {
	struct i2c_dt_spec i2c;
#ifdef CONFIG_MMC5983MA_TRIGGER
	struct gpio_dt_spec int_gpio;
#endif
	uint8_t bandwidth;
	bool auto_set_reset;
};

struct mmc5983ma_data {
	struct k_mutex lock;

	/* Shadows of the write-only control registers. */
	uint8_t ctrl0;
	uint8_t ctrl1;
	uint8_t ctrl2;

	/* Continuous mode rate in Hz, 0 = single shot. */
	uint16_t odr;

	/* Latest raw 18-bit samples and temperature code. */
	uint32_t raw[3];
	uint8_t raw_temp;
	bool temp_valid;

#ifdef CONFIG_MMC5983MA_TRIGGER
	const struct device *dev;
	struct gpio_callback gpio_cb;
	sensor_trigger_handler_t handler;
	const struct sensor_trigger *trigger;
#if defined(CONFIG_MMC5983MA_TRIGGER_OWN_THREAD)
	K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_MMC5983MA_THREAD_STACK_SIZE);
	struct k_thread thread;
	struct k_sem gpio_sem;
#elif defined(CONFIG_MMC5983MA_TRIGGER_GLOBAL_THREAD)
	struct k_work work;
#endif
#endif /* CONFIG_MMC5983MA_TRIGGER */
};

int mmc5983ma_reg_write(const struct device *dev, uint8_t reg, uint8_t val);
int mmc5983ma_reg_read(const struct device *dev, uint8_t reg, uint8_t *val);
int mmc5983ma_write_ctrl0(const struct device *dev, uint8_t extra_bits);

#ifdef CONFIG_MMC5983MA_TRIGGER
int mmc5983ma_trigger_set(const struct device *dev, const struct sensor_trigger *trig,
			  sensor_trigger_handler_t handler);
int mmc5983ma_trigger_init(const struct device *dev);
#endif

#endif /* ZEPHYR_DRIVERS_SENSOR_MEMSIC_MMC5983MA_MMC5983MA_H_ */
