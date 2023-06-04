# ifndef _EGO_MOTION_H_
# define _EGO_MOTION_H_

# include "datatypes.h"
# include "constants.h"

/* ---------------------------------------------------------------------
 *                          Structure Declaration
 * --------------------------------------------------------------------- */

typedef struct point_2d {
    REAL x;
    REAL y;
}point_2d;


typedef struct mount_param_quaternion {
    REAL tx, ty, tz;
    REAL q1, q2, q3, q4;
} mount_param_quaternion;


typedef struct mount_param {
	REAL tx, ty, theta;
	REAL cos_theta, sin_theta;
} mount_param;


typedef struct meas {
    REAL px, py;
    REAL vx, vy;
    REAL px_rms, py_rms;
    REAL vx_rms, vy_rms;
    uint8_t dyn_prop;
    uint8_t ambig_state;
    uint8_t valid_state;
} meas;


typedef struct meas_aos {
    REAL timestamp_sec;
    uint16_t num_valid_meas;
    meas rad_meas_arr[max_nrad_meas];
} meas_aos;


typedef struct errors {
    uint16_t num_valid_meas;
    point_2d error_arr[max_nrad_meas];
}errors;


typedef struct error_statistics {
    REAL error_mu[2] = {0};
    REAL error_cov[2][2] = {0};
}error_statistics;


typedef struct gated_meas_ids {
    uint16_t num_meas_gated;
    sint16_t gated_meas_idx[max_nrad_meas];
} gated_meas_ids;


typedef struct estimated_ego_motion {
    REAL timestamp_sec;
    REAL vx;
    REAL vy;
    REAL yaw_rate;
} estimated_ego_motion;

/* ---------------------------------------------------------------------
 *                 Function Prototypes : init functions
 * --------------------------------------------------------------------- */

mount_param init_struct_mounting_param(const mount_param_quaternion* calib);

meas_aos init_meas_aos();

estimated_ego_motion init_struct_estimated_ego_motion();

errors init_errors();

gated_meas_ids init_gated_meas_ids();

/* ----------------------------------------------------------------------------------------------------------
 *                 Function Prototypes : coordinate transformation
 * ---------------------------------------------------------------------------------------------------------- */

void cts_sf2vf(
    meas* rad_meas, 
    const mount_param* param);

void coordinate_transformation_sf2vf(
    meas_aos* meas, 
    const mount_param* param);

/* ----------------------------------------------------------------------------------------------------------
 *                 Function Prototypes : gatings and measurement selection functions
 * ---------------------------------------------------------------------------------------------------------- */

BOOL meas_is_stationary(const meas* rad_meas);

void identify_stationary_meas(
    const meas_aos* meas, 
    gated_meas_ids* stationary_meas);

void predicted_vxvy_and_compute_error(
    const estimated_ego_motion* ego_motion_pred,
    const meas* rad_meas,
    point_2d* error);

void compute_errors(
    const estimated_ego_motion* ego_motion_pred,
    const meas_aos* meas,
    const gated_meas_ids* stationary_meas,
    errors* error);

void compute_error_statistics(
    const errors* error,
    error_statistics* error_stat);

REAL compute_mahalanobis_dist(
    const error_statistics* error_stat,
    const point_2d* error);

void perform_gating(
    const errors* error,
    const error_statistics* error_stat,
    const gated_meas_ids* stationary_meas,
    gated_meas_ids* gated_meas);

void gate_measurements(
    const estimated_ego_motion* ego_motion_pred,
    const meas_aos* meas,
    gated_meas_ids* gated_meas,
    BOOL* start);

/* ----------------------------------------------------------------------------------------------------------
 *                 Function Definition : ego-motion computation functions
 * ---------------------------------------------------------------------------------------------------------- */

void create_system_matrix_3dof(
    REAL A[3][3],
    REAL b[3],
    REAL sigma_b[3][3],
    const gated_meas_ids* gated_meas,
    const meas* rad_meas);

/*
void create_system_matrix_2dof(
    REAL A[2][2], REAL b[2], REAL sigma_b[2][2],
    const gated_meas_ids* gated_meas,
    const meas_aos* meas);
*/

void solve_3dof(
    estimated_ego_motion* ego_motion,
    REAL A[3][3],
    REAL b[3],
    REAL sigma_b[3][3]);

//void solve_2dof(estimated_ego_motion* ego_motion, REAL A[2][2], REAL b[2], REAL sigma_b[2][2]);

void compute_ego_motion_3dof_lse(
    estimated_ego_motion* ego_motion, 
    const gated_meas_ids* gated_meas, 
    const meas_aos* meas);

# endif
