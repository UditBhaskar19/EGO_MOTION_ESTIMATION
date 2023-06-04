# ifndef _CONSTANTS_H_
# define _CONSTANTS_H_

# include "datatypes.h"

/* -------------------------------------------------------
 *               Radar constant attributes
 * ------------------------------------------------------- */

const uint8_t num_rad_const = 5;
const uint16_t max_nrad_meas = 200;
const REAL invalid = 999999.0f;
const REAL false_alarm_probability_vals[] = { -invalid, 25, 50, 75, 90, 99, 99.9, 100.0 };
const uint8_t stationary_status[] = { 1, 3, 5 };
const uint8_t moving_status[] = { 0, 2, 6 };

/* -------------------------------------------------------
 *               Math constants
 * ------------------------------------------------------- */

const REAL PI_const = 3.14159265359f;
const REAL min2sec_const = 60.0f;
const REAL sec2min_const = 1.0f / 60.0f;
const REAL diam2rad_const = 2.0f;
const REAL deg2rad_const = PI_const / 180.0f;
const REAL rad2deg_const = 180.0f / PI_const;

/* -------------------------------------------------------
 *               True False constants
 * ------------------------------------------------------- */

const BOOL TRUE = 1;
const BOOL FALSE = 0;

# endif
