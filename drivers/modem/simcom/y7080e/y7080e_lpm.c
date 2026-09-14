/*
 * Copyright (c) 2026 Anastasios Kaltakis
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "y7080e_lpm.h"

/* T3412 (GPRS timer 3) unit multipliers, indexed by the 3-bit unit. */
static const long t3412_unit_seconds[8] = {
	600,     /* 000 = 10 minutes */
	3600,    /* 001 = 1 hour */
	36000,   /* 010 = 10 hours */
	2,       /* 011 = 2 seconds */
	30,      /* 100 = 30 seconds */
	60,      /* 101 = 1 minute */
	1152000, /* 110 = 320 hours */
	-1,      /* 111 = deactivated */
};

/* T3324 (GPRS timer 2) unit multipliers. Units 011..110 are reserved. */
static const long t3324_unit_seconds[8] = {
	2,   /* 000 = 2 seconds */
	60,  /* 001 = 1 minute */
	360, /* 010 = 6 minutes */
	-1, -1, -1, -1,
	-1,  /* 111 = deactivated */
};

static int parse_octet(const char *s, uint8_t *octet_out)
{
	uint8_t octet = 0;

	if (s == NULL) {
		return -1;
	}
	while (*s == ' ') {
		s++;
	}
	if (*s == '"') {
		s++;
	}
	for (int i = 0; i < 8; i++) {
		if (s[i] != '0' && s[i] != '1') {
			return -1;
		}
		octet = (uint8_t)((octet << 1) | (uint8_t)(s[i] - '0'));
	}
	if (s[8] != '\0' && s[8] != '"') {
		return -1;
	}
	*octet_out = octet;

	return 0;
}

long y7080e_timer_decode(const char *s, bool is_tau)
{
	uint8_t octet;
	long mult;

	if (parse_octet(s, &octet) != 0) {
		return -1;
	}

	mult = is_tau ? t3412_unit_seconds[octet >> 5] : t3324_unit_seconds[octet >> 5];
	if (mult < 0) {
		return -1;
	}

	return mult * (long)(octet & 0x1FU);
}

bool y7080e_is_active_timer(const char *s)
{
	uint8_t octet;
	uint8_t unit;

	if (parse_octet(s, &octet) != 0) {
		return false;
	}

	unit = (uint8_t)(octet >> 5);

	return unit == 0U || unit == 1U || unit == 2U || unit == 7U;
}

bool y7080e_is_tau_timer(const char *s)
{
	uint8_t octet;

	return parse_octet(s, &octet) == 0;
}

int y7080e_cereg_parse_granted(char **argv, uint16_t argc, int data_idx, long *active_sec,
			       long *tau_sec)
{
	const char *active = "";
	const char *tau = "";
	int ia;
	int it;
	long a;
	long t;

	if (argv == NULL || data_idx < 2) {
		return -1;
	}

	/*
	 * Y70XX layouts (AT manual 10.2.4):
	 *   URC  (data_idx 2): tac ci AcT cause reject Active TAU  -> +5 / +6
	 *   READ (data_idx 3): lac ci AcT rac cause reject Act TAU -> +6 / +7
	 */
	ia = (data_idx >= 3) ? data_idx + 6 : data_idx + 5;
	it = ia + 1;

	if (argc >= 4U && y7080e_is_active_timer(argv[argc - 2U]) &&
	    y7080e_is_tau_timer(argv[argc - 1U])) {
		active = argv[argc - 2U];
		tau = argv[argc - 1U];
	} else if (argc > (uint16_t)ia &&
		   (y7080e_is_active_timer(argv[ia]) ||
		    (argc > (uint16_t)it && y7080e_is_tau_timer(argv[it])))) {
		active = argv[ia];
		if (argc > (uint16_t)it) {
			tau = argv[it];
		}
	} else if (argc >= 8U && y7080e_is_active_timer(argv[argc - 1U])) {
		active = argv[argc - 1U];
	}

	a = y7080e_is_active_timer(active) ? y7080e_timer_decode(active, false) : -1;
	t = y7080e_is_tau_timer(tau) ? y7080e_timer_decode(tau, true) : -1;

	if (a < 0 && t < 0) {
		return -1;
	}

	if (active_sec != NULL) {
		*active_sec = a;
	}
	if (tau_sec != NULL) {
		*tau_sec = t;
	}

	return 0;
}

int y7080e_csq_to_dbm(int rssi)
{
	if (rssi == 0) {
		return -113;
	} else if (rssi == 1) {
		return -111;
	} else if (rssi >= 2 && rssi <= 30) {
		return -113 + (2 * rssi);
	} else if (rssi == 31) {
		return -51;
	}

	return Y7080E_RSSI_UNKNOWN;
}
