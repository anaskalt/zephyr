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

/*
 * T3324 (GPRS timer 2) unit multipliers. The AT manual (10.2.2) and
 * 3GPP TS 24.008 10.5.7.3 agree: every unit other than 111 that is not
 * one of the three defined ones counts in minutes.
 */
static const long t3324_unit_seconds[8] = {
	2,   /* 000 = 2 seconds */
	60,  /* 001 = 1 minute */
	360, /* 010 = 6 minutes */
	60,  /* 011 */
	60,  /* 100 */
	60,  /* 101 */
	60,  /* 110 */
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

	/* Every unit encodes a duration, so any well formed octet will do. */
	return parse_octet(s, &octet) == 0;
}

bool y7080e_is_tau_timer(const char *s)
{
	uint8_t octet;

	return parse_octet(s, &octet) == 0;
}

/* A bare decimal of one or two digits, as <n> and <stat> are; -1 otherwise. */
static int parse_small_uint(const char *s)
{
	int v = 0;
	int n = 0;

	if (s == NULL) {
		return -1;
	}
	while (*s == ' ') {
		s++;
	}
	while (*s >= '0' && *s <= '9' && n < 3) {
		v = (v * 10) + (*s - '0');
		s++;
		n++;
	}
	while (*s == ' ') {
		s++;
	}

	return (n >= 1 && n <= 2 && *s == '\0') ? v : -1;
}

int y7080e_cereg_parse_stat(char **argv, uint16_t argc, int *data_idx)
{
	int first;
	int second;
	int stat;
	int idx;

	if (argv == NULL || argc < 2U) {
		return -1;
	}

	/*
	 * Read: <n>,<stat>,<tac>,...  URC: <stat>,<tac>,...  Neither the first
	 * field (<n> 5 and <stat> 5 collide when roaming), nor quoting, nor
	 * the field count (both tails are optional) tells them apart. The
	 * second field does: a bare <stat> 0..10 in the read form; "21B9",
	 * "0000", a quoted string or nothing in the URC.
	 */
	first = parse_small_uint(argv[1]);
	second = (argc >= 3U) ? parse_small_uint(argv[2]) : -1;

	if (first >= 0 && first <= 5 && second >= 0 && second <= 10) {
		stat = second;
		idx = 3;
	} else if (first >= 0 && first <= 10) {
		stat = first;
		idx = 2;
	} else {
		return -1;
	}

	if (data_idx != NULL) {
		*data_idx = idx;
	}

	return stat;
}

int y7080e_cereg_parse_granted(char **argv, uint16_t argc, int data_idx, long *active_sec,
			       long *tau_sec)
{
	bool has_active;
	bool has_tau;
	int ia;
	int it;
	long a;
	long t;

	if (argv == NULL || data_idx < 2 || data_idx > 3) {
		return -1;
	}

	/*
	 * Both forms carry the same fields after <stat> (AT manual 10.2.4:
	 * the read reply shows them "the same as the active report"; the
	 * <rac> of the read syntax never appears on the wire):
	 *   tac ci AcT cause_type reject_cause Active-Time Periodic-TAU
	 */
	ia = data_idx + 5;
	it = data_idx + 6;
	has_active = argc > (uint16_t)ia && y7080e_is_active_timer(argv[ia]);
	has_tau = argc > (uint16_t)it && y7080e_is_tau_timer(argv[it]);

	if (!has_active && !has_tau) {
		/* No timer fields at all: leave the last grant untouched. */
		return -1;
	}

	/* No Periodic-TAU: no extended T3412, the network's standard one runs. */
	a = has_active ? y7080e_timer_decode(argv[ia], false) : Y7080E_TIMER_ABSENT;
	t = has_tau ? y7080e_timer_decode(argv[it], true) : Y7080E_TIMER_ABSENT;

	/* Present but deactivated timers revoke an earlier grant. */
	if (active_sec != NULL) {
		*active_sec = a;
	}
	if (tau_sec != NULL) {
		*tau_sec = t;
	}

	/* PSM is granted by T3324 alone. */
	return (a >= 0) ? 0 : -1;
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
