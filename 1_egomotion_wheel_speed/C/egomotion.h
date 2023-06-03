# ifndef _EGO_MOTION_H_
# define _EGO_MOTION_H_

# include "datatypes.h"
# include "linalg.h"

/* ---------------------------------------------------------------------
 *                          Structure Declaration
 * --------------------------------------------------------------------- */

typedef struct point_2d {
    REAL x;
    REAL y;
} point_2d;

typedef struct noise_var_wheels {
    REAL variance_FL;
    REAL variance_FR;
    REAL variance_RL;
    REAL variance_RR;
} noise_var_wheels;

typedef struct zoe_spec {
    REAL wheel_radius;
    REAL wheelbase;
    REAL track_length;
    REAL steering_ratio;
    REAL term;
} zoe_spec;

typedef struct wheel_locations {
    point_2d FL_wheel_coord;
    point_2d FR_wheel_coord;
    point_2d RL_wheel_coord;
    point_2d RR_wheel_coord;
} wheel_locations;

typedef struct wheel_steer_angle {
    REAL FL_wheel_steer;
    REAL FR_wheel_steer;
    REAL RL_wheel_steer;
    REAL RR_wheel_steer;
} wheel_steer_angle;

typedef struct veh_input_signals {
    REAL timestamp_sec;
    REAL FL_wheel_speed;
    REAL FR_wheel_speed;
    REAL RR_wheel_speed;
    REAL RL_wheel_speed;
    REAL steering_angle;
} veh_input_signals;

typedef struct estimated_motion_param {
    REAL timestamp_sec;
    REAL vx;
    REAL vy;
    REAL yaw_rate;
} estimated_motion_param;

/* ---------------------------------------------------------------------
 *                          Function Prototypes
 * --------------------------------------------------------------------- */

noise_var_wheels init_struct_noise_var_wheels();

zoe_spec init_struct_zoe_spec();

wheel_locations init_wheel_locations();

wheel_steer_angle init_wheel_steer_angle();

veh_input_signals init_struct_veh_input_signals();

estimated_motion_param init_struct_est_motion_param();

uint16_t compute_number_of_gated_wheels(const BOOL* gated_wheel_idx);

void convert_meas_unit(
    veh_input_signals* inputs,
    const zoe_spec* spec);

void compute_wheel_steer_angles(
    wheel_steer_angle* wheel_steer,
    const veh_input_signals* inputs,
    const zoe_spec* spec);

REAL predict_wheel_speed(
    const estimated_motion_param* ego_motion,
    const REAL wheel_steer,
    const point_2d* wheel_loc);

BOOL is_gated(
    const estimated_motion_param* ego_motion_prev,
    const REAL wheel_speed,
    const REAL wheel_steer,
    const point_2d* wheel_loc);

void gate_wheel_speed(
    BOOL* gated_wheel_idx,
    BOOL* start,
    const estimated_motion_param* ego_motion_prev,
    const veh_input_signals* inputs,
    const wheel_steer_angle* wheel_steer,
    const wheel_locations* wheel_loc,
    const zoe_spec* spec);

void create_system_matrix(
    REAL* A,
    REAL* b,
    REAL* B,
    REAL* sigma_b,
    const BOOL* gated_wheel_idx,
    const veh_input_signals* inputs,
    const wheel_locations* wheel_loc,
    const wheel_steer_angle* wheel_steer,
    const noise_var_wheels* sigma);

void solve(
    estimated_motion_param* ego_motion,
    const matrix* struct_A,
    const vector* struct_b,
    const matrix* struct_B,
    const matrix* struct_sigma_b,
    const uint16_t num_gated_wheels);

void compute_ego_motion_lse(
    const veh_input_signals* inputs,
    const wheel_locations* wheel_loc,
    const wheel_steer_angle* wheel_steer,
    const noise_var_wheels* sigma,
    const BOOL* gated_wheel_idx,
    estimated_motion_param* ego_motion);

# endif


