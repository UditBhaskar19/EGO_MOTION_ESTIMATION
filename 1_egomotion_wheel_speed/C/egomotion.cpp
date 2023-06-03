# include <math.h>
# include "egomotion.h"
# include "constants.h"
# include "linalg.h"

/* ---------------------------------------------------------------------
 *                             Constants
 * --------------------------------------------------------------------- */

const REAL gate_threshold_const = 0.25f;

const uint16_t vec_size_const = 2 * num_wheels_const;
const uint16_t mat_rows_const = 2 * num_wheels_const;
const uint16_t mat_cols_const = 2;

const uint16_t max_rows_const = vec_size_const;
const uint16_t max_cols_const = vec_size_const;

/* ---------------------------------------------------------------------
 *                             Global Variables
 * --------------------------------------------------------------------- */

REAL A[mat_rows_const][mat_cols_const] = { 0.0f };
REAL b[vec_size_const] = { 0.0f };
REAL B[vec_size_const][vec_size_const] = { 0.0f };
REAL sigma_b[vec_size_const][vec_size_const] = { 0.0f };

REAL AtA[mat_cols_const][mat_cols_const] = {0.0f};
REAL pinv[mat_cols_const][mat_cols_const] = {0.0f};
REAL sys_mat[mat_cols_const][max_cols_const] = {0.0f};
REAL est_vector[mat_cols_const] = {0.0f};

/* ---------------------------------------------------------------------
 *                 Function Definition : init functions
 * --------------------------------------------------------------------- */

noise_var_wheels init_struct_noise_var_wheels() {
    noise_var_wheels NOISE_VAR_WHEELS;
    NOISE_VAR_WHEELS.variance_FL = sigma_FL_const * sigma_FL_const;
    NOISE_VAR_WHEELS.variance_FR = sigma_FR_const * sigma_FR_const;
    NOISE_VAR_WHEELS.variance_RL = sigma_RL_const * sigma_RL_const;
    NOISE_VAR_WHEELS.variance_RR = sigma_RR_const * sigma_RR_const;
    return NOISE_VAR_WHEELS;
}


zoe_spec init_struct_zoe_spec() {
    zoe_spec ZOE_SPEC;
    ZOE_SPEC.wheel_radius = zoe_wheel_radius_const;
    ZOE_SPEC.wheelbase = zoe_wheelbase_const;
    ZOE_SPEC.track_length = zoe_track_length_const;
    ZOE_SPEC.steering_ratio = zoe_steering_ratio_const;
    ZOE_SPEC.term = zoe_track_length_const / (2.0f * zoe_wheelbase_const);
    return ZOE_SPEC;
}


wheel_locations init_wheel_locations() {
    wheel_locations WHL_LOC;
    // wheel x coordinates
    WHL_LOC.FL_wheel_coord.x = zoe_wheelbase_const;
    WHL_LOC.FR_wheel_coord.x = zoe_wheelbase_const;
    WHL_LOC.RL_wheel_coord.x = 0.0f;
    WHL_LOC.RR_wheel_coord.x = 0.0f;
    // wheel y coordinates
    WHL_LOC.FL_wheel_coord.y = zoe_track_length_const * 0.5f;
    WHL_LOC.FR_wheel_coord.y = -zoe_track_length_const * 0.5f;
    WHL_LOC.RL_wheel_coord.y = zoe_track_length_const * 0.5f;
    WHL_LOC.RR_wheel_coord.y = -zoe_track_length_const * 0.5f;
    return WHL_LOC;
}


wheel_steer_angle init_wheel_steer_angle() {
    wheel_steer_angle WHL_STEER;
    WHL_STEER.FL_wheel_steer = 0.0f;
    WHL_STEER.FR_wheel_steer = 0.0f;
    WHL_STEER.RL_wheel_steer = 0.0f;
    WHL_STEER.RR_wheel_steer = 0.0f;
    return WHL_STEER;
}


veh_input_signals init_struct_veh_input_signals() {
    veh_input_signals VEH_INPUT_SIGNALS;
    VEH_INPUT_SIGNALS.timestamp_sec = 0.0f;
    VEH_INPUT_SIGNALS.FL_wheel_speed = 0.0f;
    VEH_INPUT_SIGNALS.FR_wheel_speed = 0.0f;
    VEH_INPUT_SIGNALS.RL_wheel_speed = 0.0f;
    VEH_INPUT_SIGNALS.RR_wheel_speed = 0.0f;
    VEH_INPUT_SIGNALS.steering_angle = 0.0f;
    return VEH_INPUT_SIGNALS;
}


estimated_motion_param init_struct_est_motion_param() {
    estimated_motion_param EST_MOTION_PARAM;
    EST_MOTION_PARAM.timestamp_sec = 0.0f;
    EST_MOTION_PARAM.vx = 0.0f;
    EST_MOTION_PARAM.vy = 0.0f;
    EST_MOTION_PARAM.yaw_rate = 0.0f;
    return EST_MOTION_PARAM;
}

/* ----------------------------------------------------------------------------------------------------------
 *                 Function Definition : input modifications & input computations
 * ---------------------------------------------------------------------------------------------------------- */

void convert_meas_unit(
    veh_input_signals* inputs, 
    const zoe_spec* spec) {

    REAL conversion_factor = diam2rad_const * PI_const * spec->wheel_radius * sec2min_const;
    inputs->FL_wheel_speed = inputs->FL_wheel_speed * conversion_factor;
    inputs->FR_wheel_speed = inputs->FR_wheel_speed * conversion_factor;
    inputs->RL_wheel_speed = inputs->RL_wheel_speed * conversion_factor;
    inputs->RR_wheel_speed = inputs->RR_wheel_speed * conversion_factor;
    inputs->steering_angle = inputs->steering_angle * deg2rad_const;
}


void compute_wheel_steer_angles(
    wheel_steer_angle* wheel_steer, 
    const veh_input_signals* inputs, 
    const zoe_spec* spec) {

    REAL cot_whl_steer_angle = 1.0f / tan(inputs->steering_angle / spec->steering_ratio);
    wheel_steer->FL_wheel_steer = atan(1.0f / (cot_whl_steer_angle + spec->term));
    wheel_steer->FR_wheel_steer = atan(1.0f / (cot_whl_steer_angle - spec->term));
    wheel_steer->RL_wheel_steer = 0.0f;
    wheel_steer->RR_wheel_steer = 0.0f;
}

/* ----------------------------------------------------------------------------------------------------------
 *                 Function Definition : gatings and measurement selection functions
 * ---------------------------------------------------------------------------------------------------------- */

REAL predict_wheel_speed(
    const estimated_motion_param* ego_motion, 
    const REAL wheel_steer, 
    const point_2d* wheel_loc) {

    REAL vx_pred = ego_motion->vx - ego_motion->yaw_rate * wheel_loc->y;
    REAL vy_pred = ego_motion->vy + ego_motion->yaw_rate * wheel_loc->x;
    REAL speed = vx_pred * cos(wheel_steer) + vy_pred * sin(wheel_steer);
    return speed;
}


BOOL is_gated(
    const estimated_motion_param* ego_motion_prev,
    const REAL wheel_speed,
    const REAL wheel_steer,
    const point_2d* wheel_loc) {

    REAL pred_speed, error;
    pred_speed = predict_wheel_speed(ego_motion_prev, wheel_steer, wheel_loc);
    error = wheel_speed - pred_speed;
    error = error < 0.0f ? -error : error;
    if (error <= gate_threshold_const) return TRUE;
    else return FALSE;
}


void gate_wheel_speed(
    BOOL* gated_wheel_idx,
    BOOL* start,
    const estimated_motion_param* ego_motion_prev, 
    const veh_input_signals *inputs, 
    const wheel_steer_angle* wheel_steer, 
    const wheel_locations* wheel_loc, 
    const zoe_spec* spec) {

    if (*start) {
        gated_wheel_idx[0] = is_gated(ego_motion_prev, inputs->FL_wheel_speed, wheel_steer->FL_wheel_steer, &(wheel_loc->FL_wheel_coord));  // gate FL wheel
        gated_wheel_idx[1] = is_gated(ego_motion_prev, inputs->FR_wheel_speed, wheel_steer->FR_wheel_steer, &(wheel_loc->FR_wheel_coord));  // gate FR wheel
        gated_wheel_idx[2] = is_gated(ego_motion_prev, inputs->RL_wheel_speed, wheel_steer->RL_wheel_steer, &(wheel_loc->RL_wheel_coord));  // gate RL wheel
        gated_wheel_idx[3] = is_gated(ego_motion_prev, inputs->RR_wheel_speed, wheel_steer->RR_wheel_steer, &(wheel_loc->RR_wheel_coord));  // gate RR wheel
    }
    else *start = TRUE;
}


uint16_t compute_number_of_gated_wheels(const BOOL* gated_wheel_idx) {
    uint16_t num_gated_whls = 0;
    for (int i = 0; i < num_wheels_const; i++)
        if (gated_wheel_idx[i]) num_gated_whls++;
    return num_gated_whls;
}

/* ----------------------------------------------------------------------------------------------------------
 *                 Function Definition : ego-motion computation functions
 * ---------------------------------------------------------------------------------------------------------- */

void create_system_matrix(
    REAL* A,
    REAL* b,
    REAL* B, 
    REAL* sigma_b,
    const BOOL* gated_wheel_idx,
    const veh_input_signals* inputs,
    const wheel_locations* wheel_loc,
    const wheel_steer_angle* wheel_steer,
    const noise_var_wheels* sigma
    ) {

    REAL wheel_speed_vec[] = { 
        inputs->FL_wheel_speed, 
        inputs->FR_wheel_speed, 
        inputs->RL_wheel_speed, 
        inputs->RR_wheel_speed };

    REAL wheel_steer_vec[] = { 
        wheel_steer->FL_wheel_steer, 
        wheel_steer->FR_wheel_steer, 
        wheel_steer->RL_wheel_steer, 
        wheel_steer->RR_wheel_steer };

    REAL wheel_loc_x[] = {
        wheel_loc->FL_wheel_coord.x,
        wheel_loc->FR_wheel_coord.x,
        wheel_loc->RL_wheel_coord.x,
        wheel_loc->RR_wheel_coord.x
    };

    REAL wheel_loc_y[] = {
        wheel_loc->FL_wheel_coord.y,
        wheel_loc->FR_wheel_coord.y,
        wheel_loc->RL_wheel_coord.y,
        wheel_loc->RR_wheel_coord.y
    };

    REAL wheel_speed_noise[] = {
        sigma->variance_FL,
        sigma->variance_FR,
        sigma->variance_RL,
        sigma->variance_RR
    };

    INTEGER valid_idx = 0, idx1, idx2;
    REAL cos_whl_steer, sin_whl_steer;

    for (int i = 0; i < num_wheels_const; i++) {
        if (gated_wheel_idx[i]) {

            // row indexes
            idx1 = 2 * valid_idx;
            idx2 = idx1 + 1;

            // cos and sin wheel steer angles
            cos_whl_steer = cos(wheel_steer_vec[i]);
            sin_whl_steer = sin(wheel_steer_vec[i]);

            // A
            A[idx1 * mat_cols_const + 0] = 1.0f;
            A[idx2 * mat_cols_const + 0] = 0.0f;
            A[idx1 * mat_cols_const + 1] = -wheel_loc_y[i];
            A[idx2 * mat_cols_const + 1] =  wheel_loc_x[i];

            // b
            b[idx1] = wheel_speed_vec[i] * cos_whl_steer;
            b[idx2] = wheel_speed_vec[i] * sin_whl_steer;

            // B
            B[idx1 * max_cols_const + idx1] = cos_whl_steer;
            B[idx2 * max_cols_const + idx2] = sin_whl_steer;

            // sigma_b
            sigma_b[idx1 * max_cols_const + idx1] = wheel_speed_noise[i];
            sigma_b[idx1 * max_cols_const + idx2] = wheel_speed_noise[i];
            sigma_b[idx2 * max_cols_const + idx1] = wheel_speed_noise[i];
            sigma_b[idx2 * max_cols_const + idx2] = wheel_speed_noise[i];

            valid_idx++;
        }
    }
}


void solve(
    estimated_motion_param* ego_motion, 
    const matrix* struct_A,
    const vector* struct_b,
    const matrix* struct_B,
    const matrix* struct_sigma_b,
    const uint16_t num_gated_wheels) {

    // valid number of max rows
    uint16_t valid_max_size = 2 * num_gated_wheels;

    // init structures
    matrix struct_AtA, struct_pinv, struct_sys_mat;
    vector struct_est_vector;

    init_matrix_struct(&struct_AtA, &AtA[0][0], mat_cols_const, mat_cols_const);
    init_matrix_struct(&struct_pinv, &pinv[0][0], mat_cols_const, mat_cols_const);
    init_matrix_struct(&struct_sys_mat, &sys_mat[0][0], mat_cols_const, max_cols_const);
    init_vector_struct(&struct_est_vector, &est_vector[0], vec_size_const);

    matrix_matrix_multiply_AtA(struct_A, valid_max_size, 2, &struct_AtA);                                          // compute B = A' @ A
    matrix_inverse(&struct_AtA, mat_cols_const, &struct_pinv);                                                     // compute pseudo-inverse C = B^(-1)
    matrix_matrix_multiply_ABt(&struct_pinv, 2, 2, struct_A, valid_max_size, 2, &struct_sys_mat);                  // compute D = C @ A'
    matrix_vector_multiply_MV(&struct_sys_mat, 2, valid_max_size, struct_b, valid_max_size, &struct_est_vector);   // compute x = D @ b

    // set the estimated value
    ego_motion->vx = est_vector[0];
    ego_motion->vy = 0.0f;
    ego_motion->yaw_rate = est_vector[1];

    // compute the estimation covariance


    // compute A'A 

    // compute (A'A)^(-1)

    // compute A <- ((A'A)(-1))A'

    // compute x = A @ b

    // compute H = A @ B

    // compute H @ sigma_b @ H'

    /*
    # perform least squares
    A = np.linalg.inv(A.transpose() @ A) @ A.transpose()

    # compute estimates
    x = A @ b
    H = A @ B
    cov = H @ sigma_b @ H.transpose()

    # output
    vx = x[0, 0]
    yaw_rate = x[1, 0]
    ego_motion = np.array([vx, yaw_rate])
    */
    
}


void compute_ego_motion_lse(
    const veh_input_signals* inputs,
    const wheel_locations* wheel_loc,
    const wheel_steer_angle* wheel_steer,
    const noise_var_wheels* sigma,
    const BOOL* gated_wheel_idx,
    estimated_motion_param* ego_motion) {

    // number of gated wheels
    uint16_t num_gated_wheels = compute_number_of_gated_wheels(gated_wheel_idx);

    // create the system matrices
    create_system_matrix(&A[0][0], &b[0], &B[0][0], &sigma_b[0][0], gated_wheel_idx, inputs, wheel_loc, wheel_steer, sigma);
    
    // create the structures
    vector struct_b;
    matrix struct_A, struct_B, struct_sigma_b;
    init_matrix_struct(&struct_A, &A[0][0], mat_rows_const, mat_cols_const);
    init_vector_struct(&struct_b, &b[0], vec_size_const);
    init_matrix_struct(&struct_B, &B[0][0], vec_size_const, vec_size_const);
    init_matrix_struct(&struct_sigma_b, &sigma_b[0][0], vec_size_const, vec_size_const);
    
    // solve the lse
    solve(ego_motion, &struct_A, &struct_b, &struct_B, &struct_sigma_b, num_gated_wheels);
    ego_motion->timestamp_sec = inputs->timestamp_sec;
}