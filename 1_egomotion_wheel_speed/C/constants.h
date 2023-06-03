# ifndef _CONSTANTS_H_
# define _CONSTANTS_H_

# include "datatypes.h"

/* -------------------------------------------------------
 *               Zoe-vehicle constants
 * ------------------------------------------------------- */

const uint16_t num_wheels_const = 4;

const REAL sigma_FL_const = 0.03f;
const REAL sigma_FR_const = 0.03f;
const REAL sigma_RL_const = 0.03f;
const REAL sigma_RR_const = 0.03f;

const REAL zoe_wheel_radius_const = 0.305f;
const REAL zoe_wheelbase_const = 2.588f;
const REAL zoe_track_length_const = 1.511f;
const REAL zoe_steering_ratio_const = 15.2f;

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

