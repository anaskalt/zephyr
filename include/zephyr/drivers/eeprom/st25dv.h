/*
 * Copyright (c) 2026 Anastasios Kaltakis
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Vendor extensions of the ST25DV dynamic NFC tag driver.
 *
 * The user memory of the tag is accessed through the generic EEPROM API
 * (eeprom_read(), eeprom_write(), eeprom_size()). Everything else the tag
 * offers to the I2C host lives here.
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_EEPROM_ST25DV_H_
#define ZEPHYR_INCLUDE_DRIVERS_EEPROM_ST25DV_H_

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Silicon variant, detected from the IC_REF register at init. */
enum st25dv_variant {
	ST25DV_VARIANT_UNKNOWN = 0,
	/** ST25DV04K / 16K / 64K (IC_REF 0x24 / 0x26). */
	ST25DV_VARIANT_K,
	/** ST25DV04KC / 16KC / 64KC (IC_REF 0x50 / 0x51). */
	ST25DV_VARIANT_KC,
};

/**
 * @name Interrupt status bits (IT_STS_Dyn), identical on both variants.
 * @{
 */
#define ST25DV_IT_RF_USER      BIT(0)
#define ST25DV_IT_RF_ACTIVITY  BIT(1)
#define ST25DV_IT_RF_INTERRUPT BIT(2)
#define ST25DV_IT_FIELD_FALLING BIT(3)
#define ST25DV_IT_FIELD_RISING BIT(4)
#define ST25DV_IT_RF_PUT_MSG   BIT(5)
#define ST25DV_IT_RF_GET_MSG   BIT(6)
#define ST25DV_IT_RF_WRITE     BIT(7)
/** @} */

/**
 * @name GPO event sources, variant independent.
 *
 * The driver maps them to the GPO register layout of the fitted die.
 * @{
 */
#define ST25DV_GPO_RF_USER      BIT(0)
#define ST25DV_GPO_RF_ACTIVITY  BIT(1)
#define ST25DV_GPO_RF_INTERRUPT BIT(2)
#define ST25DV_GPO_FIELD_CHANGE BIT(3)
#define ST25DV_GPO_RF_PUT_MSG   BIT(4)
#define ST25DV_GPO_RF_GET_MSG   BIT(5)
#define ST25DV_GPO_RF_WRITE     BIT(6)
/** @} */

/**
 * @name Selected register addresses.
 *
 * System registers are read with st25dv_read_sys_reg(), dynamic registers
 * with st25dv_read_dyn_reg().
 * @{
 */
#define ST25DV_REG_GPO          0x0000U /* K: GPO, KC: GPO1 */
#define ST25DV_REG_GPO2         0x0001U /* KC only; K: IT_TIME */
#define ST25DV_REG_EH_MODE      0x0002U
#define ST25DV_REG_RF_MNGT      0x0003U
#define ST25DV_REG_RFA1SS       0x0004U
#define ST25DV_REG_ENDA1        0x0005U
#define ST25DV_REG_RFA2SS       0x0006U
#define ST25DV_REG_ENDA2        0x0007U
#define ST25DV_REG_RFA3SS       0x0008U
#define ST25DV_REG_ENDA3        0x0009U
#define ST25DV_REG_RFA4SS       0x000AU
#define ST25DV_REG_I2CSS        0x000BU
#define ST25DV_REG_LOCK_CCFILE  0x000CU
#define ST25DV_REG_MB_MODE      0x000DU /* KC: FTM (MB_MODE + MB_WDG) */
#define ST25DV_REG_MB_WDG       0x000EU /* K only; KC: I2C_CFG */
#define ST25DV_REG_LOCK_CFG     0x000FU
#define ST25DV_REG_MEM_SIZE     0x0014U
#define ST25DV_REG_BLK_SIZE     0x0016U
#define ST25DV_REG_IC_REF       0x0017U
#define ST25DV_REG_UID          0x0018U
#define ST25DV_REG_IC_REV       0x0020U
#define ST25DV_REG_I2C_PWD      0x0900U

#define ST25DV_DYN_GPO_CTRL     0x2000U
#define ST25DV_DYN_EH_CTRL      0x2002U
#define ST25DV_DYN_RF_MNGT      0x2003U
#define ST25DV_DYN_I2C_SSO      0x2004U
#define ST25DV_DYN_IT_STS       0x2005U
#define ST25DV_DYN_MB_CTRL      0x2006U
#define ST25DV_DYN_MB_LEN       0x2007U
#define ST25DV_MAILBOX_ADDR     0x2008U
#define ST25DV_MAILBOX_SIZE     256U
/** @} */

/** EH_CTRL_Dyn bits. */
#define ST25DV_EH_CTRL_EH_EN    BIT(0)
#define ST25DV_EH_CTRL_EH_ON    BIT(1)
#define ST25DV_EH_CTRL_FIELD_ON BIT(2)
#define ST25DV_EH_CTRL_VCC_ON   BIT(3)

/** MB_CTRL_Dyn bits. */
#define ST25DV_MB_CTRL_MB_EN           BIT(0)
#define ST25DV_MB_CTRL_HOST_PUT_MSG    BIT(1)
#define ST25DV_MB_CTRL_RF_PUT_MSG      BIT(2)
#define ST25DV_MB_CTRL_HOST_MISS_MSG   BIT(4)
#define ST25DV_MB_CTRL_RF_MISS_MSG     BIT(5)
#define ST25DV_MB_CTRL_HOST_CURRENT_MSG BIT(6)
#define ST25DV_MB_CTRL_RF_CURRENT_MSG  BIT(7)

/**
 * @brief GPO event callback.
 *
 * Called from the system work queue after every burst of GPO pulses with
 * the accumulated IT_STS_Dyn bits (@ref ST25DV_IT_FIELD_RISING and friends).
 *
 * @param dev ST25DV device.
 * @param it_status Interrupt status bits.
 * @param user_data Pointer given to st25dv_set_event_callback().
 */
typedef void (*st25dv_event_cb_t)(const struct device *dev, uint8_t it_status,
				  void *user_data);

/**
 * @brief Register the GPO event callback and arm the GPO interrupt.
 *
 * @param dev ST25DV device with a gpo-gpios property.
 * @param cb Callback, or NULL to disarm the interrupt.
 * @param user_data Passed to the callback.
 * @return 0 on success, -ENOTSUP without a GPO line, negative errno otherwise.
 */
int st25dv_set_event_callback(const struct device *dev, st25dv_event_cb_t cb,
			      void *user_data);

/** @brief Silicon variant detected at init. */
enum st25dv_variant st25dv_get_variant(const struct device *dev);

/** @brief IC_REF register value read at init. */
uint8_t st25dv_get_ic_ref(const struct device *dev);

/**
 * @brief Copy the 8-byte ISO 15693 UID read at init.
 *
 * uid[0] is the least significant byte, uid[7] is always 0xE0.
 */
int st25dv_get_uid(const struct device *dev, uint8_t uid[8]);

/**
 * @brief Read the interrupt status register and clear it.
 *
 * @return 0 on success, negative errno otherwise.
 */
int st25dv_read_it_status(const struct device *dev, uint8_t *status);

/** @brief Report whether a reader field is currently present. */
int st25dv_field_present(const struct device *dev, bool *present);

/** @brief Read one system configuration register (0x0000-0x0023). */
int st25dv_read_sys_reg(const struct device *dev, uint16_t reg, uint8_t *val);

/**
 * @brief Write one system configuration register.
 *
 * Requires an open I2C security session (st25dv_present_password()).
 * Blocks for the EEPROM write cycle.
 */
int st25dv_write_sys_reg(const struct device *dev, uint16_t reg, uint8_t val);

/** @brief Read one dynamic register (0x2000-0x2007). */
int st25dv_read_dyn_reg(const struct device *dev, uint16_t reg, uint8_t *val);

/** @brief Write one dynamic register. No password needed. */
int st25dv_write_dyn_reg(const struct device *dev, uint16_t reg, uint8_t val);

/**
 * @brief Present the I2C password and open the security session.
 *
 * @param dev ST25DV device.
 * @param password 8 bytes, most significant byte first, or NULL to use
 *                 CONFIG_EEPROM_ST25DV_I2C_PASSWORD.
 * @return 0 when the session is open, -EACCES when the tag rejected the
 *         password, negative errno otherwise.
 */
int st25dv_present_password(const struct device *dev, const uint8_t *password);

/**
 * @brief Replace the I2C password.
 *
 * The session must already be open. Blocks for the EEPROM write cycle.
 */
int st25dv_write_password(const struct device *dev, const uint8_t password[8]);

/** @brief Report whether the I2C security session is open. */
int st25dv_session_open(const struct device *dev, bool *open);

/**
 * @brief Select the events reported on the GPO pin.
 *
 * Opens the security session with the configured password, writes the
 * static GPO register of the fitted variant and verifies it. Events
 * already programmed are only rewritten when they differ.
 *
 * @param dev ST25DV device.
 * @param events Bitwise OR of ST25DV_GPO_* sources.
 * @param enable Drive the GPO pin (false parks it high impedance).
 */
int st25dv_gpo_configure(const struct device *dev, uint8_t events, bool enable);

/** @brief Enable or disable the GPO output dynamically (GPO_CTRL_Dyn). */
int st25dv_gpo_output_enable(const struct device *dev, bool enable);

/**
 * @brief Enable or disable the fast transfer mode mailbox.
 *
 * Enabling also sets MB_MODE in the static FTM register when it is still
 * at its factory value (needs the security session). While the mailbox is
 * enabled no EEPROM write is possible; eeprom_write() disables it first.
 */
int st25dv_mailbox_enable(const struct device *dev, bool enable);

/** @brief Read MB_CTRL_Dyn. */
int st25dv_mailbox_status(const struct device *dev, uint8_t *ctrl);

/**
 * @brief Read the message left in the mailbox by the reader.
 *
 * @param dev ST25DV device.
 * @param buf Destination, at least ST25DV_MAILBOX_SIZE bytes for a full read.
 * @param max_len Buffer size.
 * @return The message length, or negative errno (-ENODATA when no reader
 *         message is pending).
 */
int st25dv_mailbox_read(const struct device *dev, uint8_t *buf, size_t max_len);

/**
 * @brief Put a message in the mailbox for the reader.
 *
 * @return 0 on success, -EBUSY when the mailbox still holds an unread
 *         message, negative errno otherwise.
 */
int st25dv_mailbox_write(const struct device *dev, const uint8_t *buf, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_EEPROM_ST25DV_H_ */
