/*
 * Copyright (c) 2026 Anastasios Kaltakis
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Extended public API for the MEMSIC MMC5983MA magnetometer.
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_SENSOR_MMC5983MA_H_
#define ZEPHYR_INCLUDE_DRIVERS_SENSOR_MMC5983MA_H_

#include <zephyr/drivers/sensor.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief MMC5983MA specific sensor attributes.
 *
 * All of them apply to SENSOR_CHAN_MAGN_XYZ (or SENSOR_CHAN_ALL).
 *
 * The generic SENSOR_ATTR_SAMPLING_FREQUENCY selects the continuous
 * measurement mode: 0 (single shot, the default), 1, 10, 20, 50, 100,
 * 200 or 1000 Hz. 200 Hz needs a bandwidth of at least 1 and 1000 Hz a
 * bandwidth of 3. The die temperature cannot be measured while the
 * continuous mode is running.
 */
enum mmc5983ma_attribute {
	/**
	 * Decimation filter length BW[1:0], val1 = 0..3. Trades measurement
	 * time (8, 4, 2, 0.5 ms) and supply current for noise.
	 */
	MMC5983MA_ATTR_BANDWIDTH = SENSOR_ATTR_PRIV_START,
	/** Automatic SET/RESET around every measurement, val1 = 0 or 1. */
	MMC5983MA_ATTR_AUTO_SET_RESET,
	/** Perform one SET pulse now. The value is ignored. */
	MMC5983MA_ATTR_SET,
	/** Perform one RESET pulse now. The value is ignored. */
	MMC5983MA_ATTR_RESET,
	/**
	 * Periodic SET in continuous mode: val1 = Prd_set code 0..7
	 * (1, 25, 75, 100, 250, 500, 1000, 2000 measurements), val2 = 0 to
	 * disable or 1 to enable. Needs auto SET/RESET and continuous mode.
	 */
	MMC5983MA_ATTR_PERIODIC_SET,
};

/** Number of counts per gauss in the 18-bit output. */
#define MMC5983MA_COUNTS_PER_GAUSS 16384
/** Output code of a null field in the 18-bit output. */
#define MMC5983MA_NULL_FIELD_COUNTS 131072

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_SENSOR_MMC5983MA_H_ */
