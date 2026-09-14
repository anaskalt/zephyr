/*
 * Copyright (c) 2026 Anastasios Kaltakis
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * STMicroelectronics ST25DV dynamic NFC tag: I2C EEPROM, GPO events,
 * security session and fast-transfer-mode mailbox.
 *
 * Two dies exist with the same I2C protocol but a different static GPO
 * register layout: the "K" family (DS11080) and the "KC" family (DS13519).
 * The driver detects the die from IC_REF and maps the vendor API onto it.
 *
 * The tag is addressed at two 7-bit addresses: reg (0x53) for the user
 * memory, dynamic registers and mailbox (E2 = 0) and reg | 0x04 (0x57) for
 * the system configuration area (E2 = 1). Every access carries a 16-bit
 * big-endian memory address.
 *
 * The RF side has priority: while a reader talks to the tag every I2C
 * access is NACKed, and an EEPROM write cycle (tW, 5 ms per 16-byte row)
 * also NACKs. Both cases are handled by bounded retries.
 */

#define DT_DRV_COMPAT st_st25dv

#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/eeprom.h>
#include <zephyr/drivers/eeprom/st25dv.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(eeprom_st25dv, CONFIG_EEPROM_LOG_LEVEL);

/* Timing, from the I2C AC characteristics tables. */
#define ST25DV_TBOOT_MS         2   /* tbootDC max 0.6 ms, rounded up to ticks */
#define ST25DV_TW_MS            5   /* EEPROM write cycle, max */
#define ST25DV_TW_TIMEOUT_MS    15  /* ACK-poll window per row */
#define ST25DV_BUSY_TIMEOUT_MS  80  /* RF-busy retry window per access */
#define ST25DV_BUSY_RETRY_MS    1
#define ST25DV_ROW_SIZE         16
#define ST25DV_SYS_ADDR_BIT     0x04

/* Keep VCC on this long after the last access so bursts of accesses do
 * not pay the boot delay every time.
 */
#define ST25DV_POWER_LINGER_MS  50

/* IC_REF values. */
#define ST25DV_IC_REF_04K       0x24
#define ST25DV_IC_REF_16K_64K   0x26
#define ST25DV_IC_REF_04KC      0x50
#define ST25DV_IC_REF_16KC_64KC 0x51

/* K family GPO register (0x0000). */
#define K_GPO_EN                BIT(7)
#define K_GPO_EVENT_MASK        0x7FU

/* KC family GPO1 register (0x0000). */
#define KC_GPO1_EN              BIT(0)
#define KC_GPO1_EVENT_SHIFT     1

/* MB_MODE / FTM static register. */
#define ST25DV_MB_MODE_EN       BIT(0)

/* Password commands. */
#define ST25DV_PWD_LEN          8
#define ST25DV_PWD_PRESENT_CODE 0x09
#define ST25DV_PWD_WRITE_CODE   0x07

#define ST25DV_I2C_SSO_OPEN     BIT(0)

#define ST25DV_GPO_EVENT_RETRIES 5

struct st25dv_config {
	struct i2c_dt_spec i2c;
	struct gpio_dt_spec vcc;
	struct gpio_dt_spec gpo;
	uint16_t sys_addr;
	size_t size;
	bool readonly;
};

struct st25dv_data {
	const struct device *dev;
	struct k_mutex lock;
	uint8_t tx[2 + ST25DV_MAILBOX_SIZE];
	uint8_t password[ST25DV_PWD_LEN];

	enum st25dv_variant variant;
	uint8_t ic_ref;
	uint8_t uid[8];

	struct gpio_callback gpo_cb;
	struct k_work_delayable gpo_work;
	struct k_work_delayable field_work;
	st25dv_event_cb_t event_cb;
	void *event_user;
	uint8_t event_retries;
	bool field_hold;
};

/* ------------------------------------------------------------------ */
/* Power / locking                                                     */
/* ------------------------------------------------------------------ */

static int st25dv_begin(const struct device *dev)
{
	struct st25dv_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);

	ret = pm_device_runtime_get(dev);
	if (ret < 0) {
		k_mutex_unlock(&data->lock);
	}

	return ret;
}

static void st25dv_end(const struct device *dev)
{
	struct st25dv_data *data = dev->data;

	(void)pm_device_runtime_put_async(dev, K_MSEC(ST25DV_POWER_LINGER_MS));
	k_mutex_unlock(&data->lock);
}

/* ------------------------------------------------------------------ */
/* Raw I2C                                                             */
/* ------------------------------------------------------------------ */

static int st25dv_read(const struct device *dev, bool sys, uint16_t addr,
		       uint8_t *buf, size_t len)
{
	const struct st25dv_config *cfg = dev->config;
	uint16_t i2c_addr = sys ? cfg->sys_addr : cfg->i2c.addr;
	uint8_t a[2] = {addr >> 8, addr & 0xFFU};
	int64_t deadline = k_uptime_get() + ST25DV_BUSY_TIMEOUT_MS;
	int ret;

	for (;;) {
		ret = i2c_write_read(cfg->i2c.bus, i2c_addr, a, sizeof(a), buf, len);
		if (ret == 0 || k_uptime_get() >= deadline) {
			break;
		}
		/* RF busy or write cycle in progress: the device NACKs. */
		k_sleep(K_MSEC(ST25DV_BUSY_RETRY_MS));
	}

	if (ret < 0) {
		LOG_DBG("read %s@0x%04x len %u failed (%d)", sys ? "sys" : "usr",
			addr, (unsigned int)len, ret);
	}

	return ret;
}

/* Send a write transaction. The caller has filled data->tx[2..] with len
 * bytes; the 16-bit address is prepended here.
 */
static int st25dv_write_raw(const struct device *dev, bool sys, uint16_t addr,
			    size_t len)
{
	const struct st25dv_config *cfg = dev->config;
	struct st25dv_data *data = dev->data;
	uint16_t i2c_addr = sys ? cfg->sys_addr : cfg->i2c.addr;
	int64_t deadline = k_uptime_get() + ST25DV_BUSY_TIMEOUT_MS;
	int ret;

	data->tx[0] = addr >> 8;
	data->tx[1] = addr & 0xFFU;

	for (;;) {
		ret = i2c_write(cfg->i2c.bus, data->tx, 2 + len, i2c_addr);
		if (ret == 0 || k_uptime_get() >= deadline) {
			break;
		}
		k_sleep(K_MSEC(ST25DV_BUSY_RETRY_MS));
	}

	if (ret < 0) {
		LOG_DBG("write %s@0x%04x len %u failed (%d)", sys ? "sys" : "usr",
			addr, (unsigned int)len, ret);
	}

	return ret;
}

/* Wait for the EEPROM write cycle by ACK polling the device select. A
 * controller that rejects zero-length transfers falls back to the maximum
 * tW.
 */
static int st25dv_wait_write_cycle(const struct device *dev, bool sys)
{
	const struct st25dv_config *cfg = dev->config;
	uint16_t i2c_addr = sys ? cfg->sys_addr : cfg->i2c.addr;
	uint8_t dummy = 0;
	struct i2c_msg msg = {
		.buf = &dummy,
		.len = 0,
		.flags = I2C_MSG_WRITE | I2C_MSG_STOP,
	};
	int64_t deadline = k_uptime_get() + ST25DV_TW_TIMEOUT_MS;
	int ret;

	k_sleep(K_MSEC(1));

	for (;;) {
		ret = i2c_transfer(cfg->i2c.bus, &msg, 1, i2c_addr);
		if (ret == 0) {
			return 0;
		}
		if (ret == -ENOTSUP || ret == -EINVAL) {
			k_sleep(K_MSEC(ST25DV_TW_MS));
			return 0;
		}
		if (k_uptime_get() >= deadline) {
			return -ETIMEDOUT;
		}
		k_sleep(K_MSEC(1));
	}
}

/* Write one EEPROM byte (system register) and wait for the cycle. */
static int st25dv_write_sys_byte(const struct device *dev, uint16_t reg, uint8_t val)
{
	struct st25dv_data *data = dev->data;
	int ret;

	data->tx[2] = val;
	ret = st25dv_write_raw(dev, true, reg, 1);
	if (ret < 0) {
		return ret;
	}

	return st25dv_wait_write_cycle(dev, true);
}

static int st25dv_write_dyn_byte(const struct device *dev, uint16_t reg, uint8_t val)
{
	struct st25dv_data *data = dev->data;

	data->tx[2] = val;
	return st25dv_write_raw(dev, false, reg, 1);
}

static int st25dv_read_byte(const struct device *dev, bool sys, uint16_t reg, uint8_t *val)
{
	return st25dv_read(dev, sys, reg, val, 1);
}

/* Make sure the mailbox is disabled: every EEPROM write transits through
 * the 256-byte buffer and is NACKed while fast transfer mode is enabled.
 */
static int st25dv_mailbox_off_locked(const struct device *dev)
{
	uint8_t ctrl;
	int ret;

	ret = st25dv_read_byte(dev, false, ST25DV_DYN_MB_CTRL, &ctrl);
	if (ret < 0) {
		return ret;
	}

	if ((ctrl & ST25DV_MB_CTRL_MB_EN) == 0U) {
		return 0;
	}

	return st25dv_write_dyn_byte(dev, ST25DV_DYN_MB_CTRL, 0);
}

static int st25dv_present_password_locked(const struct device *dev, const uint8_t *pwd)
{
	struct st25dv_data *data = dev->data;
	uint8_t sso;
	int ret;

	memcpy(&data->tx[2], pwd, ST25DV_PWD_LEN);
	data->tx[2 + ST25DV_PWD_LEN] = ST25DV_PWD_PRESENT_CODE;
	memcpy(&data->tx[3 + ST25DV_PWD_LEN], pwd, ST25DV_PWD_LEN);

	ret = st25dv_write_raw(dev, true, ST25DV_REG_I2C_PWD, 2 * ST25DV_PWD_LEN + 1);
	if (ret < 0) {
		return ret;
	}

	ret = st25dv_read_byte(dev, false, ST25DV_DYN_I2C_SSO, &sso);
	if (ret < 0) {
		return ret;
	}

	return (sso & ST25DV_I2C_SSO_OPEN) ? 0 : -EACCES;
}

/* ------------------------------------------------------------------ */
/* EEPROM API                                                          */
/* ------------------------------------------------------------------ */

static int st25dv_eeprom_read(const struct device *dev, off_t offset, void *buf, size_t len)
{
	const struct st25dv_config *cfg = dev->config;
	int ret;

	if (len == 0) {
		return 0;
	}

	if (offset < 0 || (size_t)offset + len > cfg->size) {
		return -EINVAL;
	}

	ret = st25dv_begin(dev);
	if (ret < 0) {
		return ret;
	}

	ret = st25dv_read(dev, false, (uint16_t)offset, buf, len);

	st25dv_end(dev);

	return ret;
}

static int st25dv_eeprom_write(const struct device *dev, off_t offset, const void *buf,
			       size_t len)
{
	const struct st25dv_config *cfg = dev->config;
	struct st25dv_data *data = dev->data;
	const uint8_t *src = buf;
	int ret;

	if (cfg->readonly) {
		return -EACCES;
	}

	if (len == 0) {
		return 0;
	}

	if (offset < 0 || (size_t)offset + len > cfg->size) {
		return -EINVAL;
	}

	ret = st25dv_begin(dev);
	if (ret < 0) {
		return ret;
	}

	ret = st25dv_mailbox_off_locked(dev);

	/* One transaction per 16-byte row: each row costs one tW anyway, and
	 * a row never crosses a user-area border.
	 */
	while (ret == 0 && len > 0) {
		size_t in_row = ST25DV_ROW_SIZE - ((size_t)offset % ST25DV_ROW_SIZE);
		size_t chunk = MIN(len, in_row);

		memcpy(&data->tx[2], src, chunk);
		ret = st25dv_write_raw(dev, false, (uint16_t)offset, chunk);
		if (ret == 0) {
			ret = st25dv_wait_write_cycle(dev, false);
		}

		offset += chunk;
		src += chunk;
		len -= chunk;
	}

	st25dv_end(dev);

	return ret;
}

static size_t st25dv_eeprom_size(const struct device *dev)
{
	const struct st25dv_config *cfg = dev->config;

	return cfg->size;
}

/* ------------------------------------------------------------------ */
/* Vendor API                                                          */
/* ------------------------------------------------------------------ */

enum st25dv_variant st25dv_get_variant(const struct device *dev)
{
	struct st25dv_data *data = dev->data;

	return data->variant;
}

uint8_t st25dv_get_ic_ref(const struct device *dev)
{
	struct st25dv_data *data = dev->data;

	return data->ic_ref;
}

int st25dv_get_uid(const struct device *dev, uint8_t uid[8])
{
	struct st25dv_data *data = dev->data;

	if (data->variant == ST25DV_VARIANT_UNKNOWN && data->ic_ref == 0U) {
		return -ENODEV;
	}

	memcpy(uid, data->uid, sizeof(data->uid));

	return 0;
}

int st25dv_read_it_status(const struct device *dev, uint8_t *status)
{
	int ret;

	ret = st25dv_begin(dev);
	if (ret < 0) {
		return ret;
	}

	ret = st25dv_read_byte(dev, false, ST25DV_DYN_IT_STS, status);

	st25dv_end(dev);

	return ret;
}

int st25dv_field_present(const struct device *dev, bool *present)
{
	uint8_t eh;
	int ret;

	ret = st25dv_begin(dev);
	if (ret < 0) {
		return ret;
	}

	ret = st25dv_read_byte(dev, false, ST25DV_DYN_EH_CTRL, &eh);

	st25dv_end(dev);

	if (ret == 0) {
		*present = (eh & ST25DV_EH_CTRL_FIELD_ON) != 0U;
	}

	return ret;
}

int st25dv_read_sys_reg(const struct device *dev, uint16_t reg, uint8_t *val)
{
	int ret;

	ret = st25dv_begin(dev);
	if (ret < 0) {
		return ret;
	}

	ret = st25dv_read_byte(dev, true, reg, val);

	st25dv_end(dev);

	return ret;
}

int st25dv_write_sys_reg(const struct device *dev, uint16_t reg, uint8_t val)
{
	int ret;

	ret = st25dv_begin(dev);
	if (ret < 0) {
		return ret;
	}

	ret = st25dv_mailbox_off_locked(dev);
	if (ret == 0) {
		ret = st25dv_write_sys_byte(dev, reg, val);
	}

	st25dv_end(dev);

	return ret;
}

int st25dv_read_dyn_reg(const struct device *dev, uint16_t reg, uint8_t *val)
{
	int ret;

	ret = st25dv_begin(dev);
	if (ret < 0) {
		return ret;
	}

	ret = st25dv_read_byte(dev, false, reg, val);

	st25dv_end(dev);

	return ret;
}

int st25dv_write_dyn_reg(const struct device *dev, uint16_t reg, uint8_t val)
{
	int ret;

	ret = st25dv_begin(dev);
	if (ret < 0) {
		return ret;
	}

	ret = st25dv_write_dyn_byte(dev, reg, val);

	st25dv_end(dev);

	return ret;
}

int st25dv_present_password(const struct device *dev, const uint8_t *password)
{
	struct st25dv_data *data = dev->data;
	int ret;

	if (password == NULL) {
		password = data->password;
	}

	ret = st25dv_begin(dev);
	if (ret < 0) {
		return ret;
	}

	ret = st25dv_present_password_locked(dev, password);

	st25dv_end(dev);

	return ret;
}

int st25dv_write_password(const struct device *dev, const uint8_t password[8])
{
	struct st25dv_data *data = dev->data;
	int ret;

	ret = st25dv_begin(dev);
	if (ret < 0) {
		return ret;
	}

	ret = st25dv_mailbox_off_locked(dev);
	if (ret == 0) {
		memcpy(&data->tx[2], password, ST25DV_PWD_LEN);
		data->tx[2 + ST25DV_PWD_LEN] = ST25DV_PWD_WRITE_CODE;
		memcpy(&data->tx[3 + ST25DV_PWD_LEN], password, ST25DV_PWD_LEN);

		ret = st25dv_write_raw(dev, true, ST25DV_REG_I2C_PWD, 2 * ST25DV_PWD_LEN + 1);
		if (ret == 0) {
			ret = st25dv_wait_write_cycle(dev, true);
		}
		if (ret == 0) {
			memcpy(data->password, password, ST25DV_PWD_LEN);
		}
	}

	st25dv_end(dev);

	return ret;
}

int st25dv_session_open(const struct device *dev, bool *open)
{
	uint8_t sso;
	int ret;

	ret = st25dv_read_dyn_reg(dev, ST25DV_DYN_I2C_SSO, &sso);
	if (ret == 0) {
		*open = (sso & ST25DV_I2C_SSO_OPEN) != 0U;
	}

	return ret;
}

static uint8_t st25dv_gpo_reg_value(enum st25dv_variant variant, uint8_t events, bool enable)
{
	events &= K_GPO_EVENT_MASK;

	if (variant == ST25DV_VARIANT_KC) {
		return (uint8_t)((events << KC_GPO1_EVENT_SHIFT) | (enable ? KC_GPO1_EN : 0U));
	}

	/* K layout; also the best guess for an unknown die. */
	return (uint8_t)(events | (enable ? K_GPO_EN : 0U));
}

int st25dv_gpo_configure(const struct device *dev, uint8_t events, bool enable)
{
	struct st25dv_data *data = dev->data;
	uint8_t want, have;
	int ret;

	want = st25dv_gpo_reg_value(data->variant, events, enable);

	ret = st25dv_begin(dev);
	if (ret < 0) {
		return ret;
	}

	ret = st25dv_read_byte(dev, true, ST25DV_REG_GPO, &have);
	if (ret < 0 || have == want) {
		goto out;
	}

	ret = st25dv_mailbox_off_locked(dev);
	if (ret < 0) {
		goto out;
	}

	ret = st25dv_present_password_locked(dev, data->password);
	if (ret < 0) {
		LOG_WRN("GPO configuration needs the I2C password (%d)", ret);
		goto out;
	}

	ret = st25dv_write_sys_byte(dev, ST25DV_REG_GPO, want);
	if (ret < 0) {
		goto out;
	}

	ret = st25dv_read_byte(dev, true, ST25DV_REG_GPO, &have);
	if (ret == 0 && have != want) {
		LOG_ERR("GPO register readback 0x%02x, wanted 0x%02x", have, want);
		ret = -EIO;
	} else if (ret == 0) {
		LOG_INF("GPO register set to 0x%02x", want);
	}

out:
	st25dv_end(dev);

	return ret;
}

int st25dv_gpo_output_enable(const struct device *dev, bool enable)
{
	return st25dv_write_dyn_reg(dev, ST25DV_DYN_GPO_CTRL, enable ? 1U : 0U);
}

int st25dv_mailbox_enable(const struct device *dev, bool enable)
{
	struct st25dv_data *data = dev->data;
	uint8_t val;
	int ret;

	ret = st25dv_begin(dev);
	if (ret < 0) {
		return ret;
	}

	if (!enable) {
		ret = st25dv_write_dyn_byte(dev, ST25DV_DYN_MB_CTRL, 0);
		goto out;
	}

	ret = st25dv_read_byte(dev, true, ST25DV_REG_MB_MODE, &val);
	if (ret < 0) {
		goto out;
	}

	if ((val & ST25DV_MB_MODE_EN) == 0U) {
		ret = st25dv_present_password_locked(dev, data->password);
		if (ret < 0) {
			goto out;
		}
		ret = st25dv_write_sys_byte(dev, ST25DV_REG_MB_MODE, val | ST25DV_MB_MODE_EN);
		if (ret < 0) {
			goto out;
		}
	}

	ret = st25dv_write_dyn_byte(dev, ST25DV_DYN_MB_CTRL, ST25DV_MB_CTRL_MB_EN);
	if (ret < 0) {
		goto out;
	}

	ret = st25dv_read_byte(dev, false, ST25DV_DYN_MB_CTRL, &val);
	if (ret == 0 && (val & ST25DV_MB_CTRL_MB_EN) == 0U) {
		/* Also the case when VCC is absent: the mailbox needs it. */
		ret = -EIO;
	}

out:
	st25dv_end(dev);

	return ret;
}

int st25dv_mailbox_status(const struct device *dev, uint8_t *ctrl)
{
	return st25dv_read_dyn_reg(dev, ST25DV_DYN_MB_CTRL, ctrl);
}

int st25dv_mailbox_read(const struct device *dev, uint8_t *buf, size_t max_len)
{
	struct st25dv_data *data = dev->data;
	uint8_t ctrl, len_reg;
	size_t len;
	int ret;

	ret = st25dv_begin(dev);
	if (ret < 0) {
		return ret;
	}

	ret = st25dv_read_byte(dev, false, ST25DV_DYN_MB_CTRL, &ctrl);
	if (ret < 0) {
		goto out;
	}

	if ((ctrl & ST25DV_MB_CTRL_RF_PUT_MSG) == 0U) {
		ret = -ENODATA;
		goto out;
	}

	ret = st25dv_read_byte(dev, false, ST25DV_DYN_MB_LEN, &len_reg);
	if (ret < 0) {
		goto out;
	}

	len = (size_t)len_reg + 1U;

	/* Always read the whole message: RF_PUT_MSG is only released once the
	 * last byte has been read.
	 */
	ret = st25dv_read(dev, false, ST25DV_MAILBOX_ADDR, data->tx, len);
	if (ret < 0) {
		goto out;
	}

	memcpy(buf, data->tx, MIN(len, max_len));
	ret = (int)len;

out:
	st25dv_end(dev);

	return ret;
}

int st25dv_mailbox_write(const struct device *dev, const uint8_t *buf, size_t len)
{
	struct st25dv_data *data = dev->data;
	uint8_t ctrl;
	int ret;

	if (len == 0 || len > ST25DV_MAILBOX_SIZE) {
		return -EINVAL;
	}

	ret = st25dv_begin(dev);
	if (ret < 0) {
		return ret;
	}

	ret = st25dv_read_byte(dev, false, ST25DV_DYN_MB_CTRL, &ctrl);
	if (ret < 0) {
		goto out;
	}

	if ((ctrl & ST25DV_MB_CTRL_MB_EN) == 0U) {
		ret = -EACCES;
		goto out;
	}

	if ((ctrl & (ST25DV_MB_CTRL_HOST_PUT_MSG | ST25DV_MB_CTRL_RF_PUT_MSG)) != 0U) {
		ret = -EBUSY;
		goto out;
	}

	memcpy(&data->tx[2], buf, len);
	ret = st25dv_write_raw(dev, false, ST25DV_MAILBOX_ADDR, len);

out:
	st25dv_end(dev);

	return ret;
}

/* ------------------------------------------------------------------ */
/* GPO events                                                          */
/* ------------------------------------------------------------------ */

static void st25dv_field_release(const struct device *dev)
{
	struct st25dv_data *data = dev->data;

	if (data->field_hold) {
		data->field_hold = false;
		(void)pm_device_runtime_put_async(dev, K_MSEC(ST25DV_POWER_LINGER_MS));
	}
}

static void st25dv_gpo_work_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct st25dv_data *data = CONTAINER_OF(dwork, struct st25dv_data, gpo_work);
	const struct device *dev = data->dev;
	uint8_t sts = 0;
	int ret;

	ret = st25dv_read_it_status(dev, &sts);
	if (ret < 0) {
		if (data->event_retries < ST25DV_GPO_EVENT_RETRIES) {
			data->event_retries++;
			k_work_reschedule(&data->gpo_work,
					  K_MSEC(CONFIG_EEPROM_ST25DV_EVENT_SETTLE_MS));
		} else {
			LOG_WRN("GPO event lost, status read failed (%d)", ret);
			data->event_retries = 0;
		}
		return;
	}
	data->event_retries = 0;

	if (sts == 0U) {
		return;
	}

	LOG_DBG("IT_STS 0x%02x", sts);

	/* The field-falling pulse only exists while VCC is present, so keep
	 * the tag powered for the whole reader session.
	 */
	if ((sts & ST25DV_IT_FIELD_RISING) != 0U && !data->field_hold) {
		if (pm_device_runtime_get(dev) == 0) {
			data->field_hold = true;
			k_work_reschedule(&data->field_work,
					  K_SECONDS(CONFIG_EEPROM_ST25DV_FIELD_TIMEOUT_S));
		}
	}

	if (data->event_cb != NULL) {
		data->event_cb(dev, sts, data->event_user);
	}

	if ((sts & ST25DV_IT_FIELD_FALLING) != 0U) {
		k_work_cancel_delayable(&data->field_work);
		st25dv_field_release(dev);
	}
}

static void st25dv_field_work_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct st25dv_data *data = CONTAINER_OF(dwork, struct st25dv_data, field_work);
	const struct device *dev = data->dev;
	bool present = false;

	if (!data->field_hold) {
		return;
	}

	if (st25dv_field_present(dev, &present) == 0 && present) {
		k_work_reschedule(&data->field_work,
				  K_SECONDS(CONFIG_EEPROM_ST25DV_FIELD_TIMEOUT_S));
		return;
	}

	LOG_DBG("field gone without a falling pulse");

	if (data->event_cb != NULL) {
		data->event_cb(dev, ST25DV_IT_FIELD_FALLING, data->event_user);
	}

	st25dv_field_release(dev);
}

static void st25dv_gpo_isr(const struct device *port, struct gpio_callback *cb, uint32_t pins)
{
	struct st25dv_data *data = CONTAINER_OF(cb, struct st25dv_data, gpo_cb);

	ARG_UNUSED(port);
	ARG_UNUSED(pins);

	k_work_reschedule(&data->gpo_work, K_MSEC(CONFIG_EEPROM_ST25DV_EVENT_SETTLE_MS));
}

int st25dv_set_event_callback(const struct device *dev, st25dv_event_cb_t cb, void *user_data)
{
	const struct st25dv_config *cfg = dev->config;
	struct st25dv_data *data = dev->data;
	int ret;

	if (cfg->gpo.port == NULL) {
		return -ENOTSUP;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	data->event_cb = cb;
	data->event_user = user_data;
	k_mutex_unlock(&data->lock);

	ret = gpio_pin_interrupt_configure_dt(&cfg->gpo,
					      cb ? GPIO_INT_EDGE_TO_ACTIVE : GPIO_INT_DISABLE);
	if (ret < 0) {
		return ret;
	}

	if (cb == NULL) {
		k_work_cancel_delayable(&data->gpo_work);
		k_work_cancel_delayable(&data->field_work);
		st25dv_field_release(dev);
	}

	return 0;
}

/* ------------------------------------------------------------------ */
/* Init                                                                */
/* ------------------------------------------------------------------ */

static int st25dv_pm_action(const struct device *dev, enum pm_device_action action)
{
	const struct st25dv_config *cfg = dev->config;
	int ret = 0;

	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		if (cfg->vcc.port != NULL) {
			ret = gpio_pin_set_dt(&cfg->vcc, 1);
			k_sleep(K_MSEC(ST25DV_TBOOT_MS));
		}
		break;
	case PM_DEVICE_ACTION_SUSPEND:
		if (cfg->vcc.port != NULL) {
			ret = gpio_pin_set_dt(&cfg->vcc, 0);
		}
		break;
	case PM_DEVICE_ACTION_TURN_ON:
	case PM_DEVICE_ACTION_TURN_OFF:
		break;
	default:
		return -ENOTSUP;
	}

	return ret;
}

static int st25dv_parse_password(const char *hex, uint8_t out[ST25DV_PWD_LEN])
{
	if (strlen(hex) != 2 * ST25DV_PWD_LEN) {
		return -EINVAL;
	}

	for (size_t i = 0; i < ST25DV_PWD_LEN; i++) {
		char byte[3] = {hex[2 * i], hex[2 * i + 1], '\0'};
		char *end;
		long v = strtol(byte, &end, 16);

		if (*end != '\0') {
			return -EINVAL;
		}
		out[i] = (uint8_t)v;
	}

	return 0;
}

static int st25dv_identify(const struct device *dev)
{
	struct st25dv_data *data = dev->data;
	int ret;

	ret = st25dv_begin(dev);
	if (ret < 0) {
		return ret;
	}

	ret = st25dv_read_byte(dev, true, ST25DV_REG_IC_REF, &data->ic_ref);
	if (ret == 0) {
		ret = st25dv_read(dev, true, ST25DV_REG_UID, data->uid, sizeof(data->uid));
	}

	st25dv_end(dev);

	if (ret < 0) {
		return ret;
	}

	switch (data->ic_ref) {
	case ST25DV_IC_REF_04K:
	case ST25DV_IC_REF_16K_64K:
		data->variant = ST25DV_VARIANT_K;
		break;
	case ST25DV_IC_REF_04KC:
	case ST25DV_IC_REF_16KC_64KC:
		data->variant = ST25DV_VARIANT_KC;
		break;
	default:
		data->variant = ST25DV_VARIANT_UNKNOWN;
		LOG_WRN("unknown IC_REF 0x%02x, assuming the K register layout", data->ic_ref);
		break;
	}

	if (data->uid[7] != 0xE0U || data->uid[6] != 0x02U) {
		LOG_WRN("UID %02x%02x%02x%02x%02x%02x%02x%02x is not an ST ISO 15693 UID",
			data->uid[7], data->uid[6], data->uid[5], data->uid[4], data->uid[3],
			data->uid[2], data->uid[1], data->uid[0]);
	}

	LOG_INF("ST25DV%s IC_REF 0x%02x UID %02x%02x%02x%02x%02x%02x%02x%02x",
		data->variant == ST25DV_VARIANT_KC ? "xxKC" : "xxK", data->ic_ref, data->uid[7],
		data->uid[6], data->uid[5], data->uid[4], data->uid[3], data->uid[2], data->uid[1],
		data->uid[0]);

	return 0;
}

static int st25dv_init(const struct device *dev)
{
	const struct st25dv_config *cfg = dev->config;
	struct st25dv_data *data = dev->data;
	int ret;

	data->dev = dev;
	k_mutex_init(&data->lock);
	k_work_init_delayable(&data->gpo_work, st25dv_gpo_work_handler);
	k_work_init_delayable(&data->field_work, st25dv_field_work_handler);

	if (!i2c_is_ready_dt(&cfg->i2c)) {
		LOG_ERR("I2C bus not ready");
		return -ENODEV;
	}

	ret = st25dv_parse_password(CONFIG_EEPROM_ST25DV_I2C_PASSWORD, data->password);
	if (ret < 0) {
		LOG_ERR("CONFIG_EEPROM_ST25DV_I2C_PASSWORD must be 16 hex digits");
		return ret;
	}

	if (cfg->vcc.port != NULL) {
		if (!gpio_is_ready_dt(&cfg->vcc)) {
			return -ENODEV;
		}
		ret = gpio_pin_configure_dt(&cfg->vcc, GPIO_OUTPUT_INACTIVE);
		if (ret < 0) {
			return ret;
		}
	}

	if (cfg->gpo.port != NULL) {
		if (!gpio_is_ready_dt(&cfg->gpo)) {
			return -ENODEV;
		}
		ret = gpio_pin_configure_dt(&cfg->gpo, GPIO_INPUT);
		if (ret < 0) {
			return ret;
		}
		gpio_init_callback(&data->gpo_cb, st25dv_gpo_isr, BIT(cfg->gpo.pin));
		ret = gpio_add_callback(cfg->gpo.port, &data->gpo_cb);
		if (ret < 0) {
			return ret;
		}
	}

	ret = pm_device_driver_init(dev, st25dv_pm_action);
	if (ret < 0) {
		return ret;
	}

	ret = st25dv_identify(dev);
	if (ret < 0) {
		LOG_ERR("tag not responding (%d)", ret);
		return ret;
	}

	return 0;
}

static DEVICE_API(eeprom, st25dv_eeprom_api) = {
	.read = st25dv_eeprom_read,
	.write = st25dv_eeprom_write,
	.size = st25dv_eeprom_size,
};

#define ST25DV_INIT(inst)                                                                          \
	static struct st25dv_data st25dv_data_##inst;                                              \
	static const struct st25dv_config st25dv_config_##inst = {                                 \
		.i2c = I2C_DT_SPEC_INST_GET(inst),                                                 \
		.vcc = GPIO_DT_SPEC_INST_GET_OR(inst, vcc_gpios, {0}),                             \
		.gpo = GPIO_DT_SPEC_INST_GET_OR(inst, gpo_gpios, {0}),                             \
		.sys_addr = DT_INST_REG_ADDR(inst) | ST25DV_SYS_ADDR_BIT,                          \
		.size = DT_INST_PROP(inst, size),                                                  \
		.readonly = DT_INST_PROP(inst, read_only),                                         \
	};                                                                                         \
	PM_DEVICE_DT_INST_DEFINE(inst, st25dv_pm_action);                                          \
	DEVICE_DT_INST_DEFINE(inst, st25dv_init, PM_DEVICE_DT_INST_GET(inst), &st25dv_data_##inst, \
			      &st25dv_config_##inst, POST_KERNEL,                                  \
			      CONFIG_EEPROM_ST25DV_INIT_PRIORITY, &st25dv_eeprom_api);

DT_INST_FOREACH_STATUS_OKAY(ST25DV_INIT)
