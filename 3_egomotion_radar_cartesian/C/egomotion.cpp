# include <math.h>
# include <stdio.h>
# include "egomotion.h"
# include "linalg.h"

/* ---------------------------------------------------------------------
 *                             Constants
 * --------------------------------------------------------------------- */

const REAL sig_const = 1.3;
const REAL chi_sq_const = sig_const * sig_const;

/* ---------------------------------------------------------------------
 *                 Function Definition : init functions
 * --------------------------------------------------------------------- */

mount_param init_struct_mounting_param(const mount_param_quaternion* calib) {

	mount_param param;

    REAL temp1 = 2 * (calib->q1 * calib->q2 + calib->q3 * calib->q4);
    REAL temp2 = 1 - 2 * (calib->q2 * calib->q2 + calib->q3 * calib->q3);
    REAL temp3 = 2 * (calib->q1 * calib->q3 - calib->q4 * calib->q2);
    REAL temp4 = 2 * (calib->q1 * calib->q4 + calib->q2 * calib->q3);
    REAL temp5 = 1 - 2 * (calib->q3 * calib->q3 + calib->q4 * calib->q4);

    REAL roll = atan2(temp1, temp2);
    REAL pitch = asin(temp3);
    REAL yaw = atan2(temp4, temp5);

    param.tx = calib->tx;
    param.ty = calib->ty;
    param.theta = yaw;
    param.cos_theta = cos(yaw);
    param.sin_theta = sin(yaw);

	return param;
}


meas_aos init_meas_aos() {
    meas_aos rad_meas;
    rad_meas.timestamp_sec = -1;
    rad_meas.num_valid_meas = 0;
    for (uint16_t i = 0; i < max_nrad_meas; i++) {
        rad_meas.rad_meas_arr[i].px = 0;
        rad_meas.rad_meas_arr[i].py = 0;
        rad_meas.rad_meas_arr[i].vx = 0;
        rad_meas.rad_meas_arr[i].vy = 0;
        rad_meas.rad_meas_arr[i].px_rms = 0;
        rad_meas.rad_meas_arr[i].py_rms = 0;
        rad_meas.rad_meas_arr[i].vx_rms = 0;
        rad_meas.rad_meas_arr[i].vy_rms = 0;
        rad_meas.rad_meas_arr[i].dyn_prop = 0;
        rad_meas.rad_meas_arr[i].ambig_state = 0;
        rad_meas.rad_meas_arr[i].valid_state = 0;
    }
    return rad_meas;
}


errors init_errors() {
    errors error;
    error.num_valid_meas = 0;
    for (uint16_t i = 0; i < max_nrad_meas; i++) {
        error.error_arr[i].x = -1;
        error.error_arr[i].y = -1;
    }
    return error;
}


gated_meas_ids init_gated_meas_ids() {
    gated_meas_ids meas_ids;
    meas_ids.num_meas_gated = 0;
    for (uint16_t i = 0; i < max_nrad_meas; i++) {
        meas_ids.gated_meas_idx[i] = -1;
    }
    return meas_ids;
}


estimated_ego_motion init_struct_estimated_ego_motion() {
    estimated_ego_motion ego_motion;
    ego_motion.timestamp_sec = -1;
    ego_motion.vx = 0;
    ego_motion.vy = 0;
    ego_motion.yaw_rate = 0;
    return ego_motion;
} 

/* ----------------------------------------------------------------------------------------------------------
 *                 Function Definition : coordinate transformation
 * ---------------------------------------------------------------------------------------------------------- */

void cts_sf2vf(
    meas* rad_meas, 
    const mount_param* param) {

    REAL px_cts = rad_meas->px * param->cos_theta - rad_meas->py * param->sin_theta + param->tx;
    REAL py_cts = rad_meas->px * param->sin_theta + rad_meas->py * param->cos_theta + param->ty;
    REAL vx_cts = rad_meas->vx * param->cos_theta - rad_meas->vy * param->sin_theta;
    REAL vy_cts = rad_meas->vx * param->sin_theta + rad_meas->vy * param->cos_theta;
    rad_meas->px = px_cts;
    rad_meas->py = py_cts;
    rad_meas->vx = vx_cts;
    rad_meas->vy = vy_cts;
}


void coordinate_transformation_sf2vf(
    meas_aos* meas, 
    const mount_param* param) {

    for (uint16_t i = 0; i < meas->num_valid_meas; i++) {
        cts_sf2vf(&meas->rad_meas_arr[i], param);
    }
}

/* ----------------------------------------------------------------------------------------------------------
 *                 Function Definition : gatings and measurement selection functions
 * ---------------------------------------------------------------------------------------------------------- */

BOOL meas_is_stationary(const meas* rad_meas) {
    return rad_meas->valid_state == (uint8_t)(0) && 
         ( rad_meas->dyn_prop == (uint8_t)(1) || rad_meas->dyn_prop == (uint8_t)(3) || rad_meas->dyn_prop == (uint8_t)(5) );
}


void identify_stationary_meas(
    const meas_aos* meas, 
    gated_meas_ids* stationary_meas) {

    uint16_t i, num_meas = 0;
    for (i = 0; i < meas->num_valid_meas; i++) {
        if (meas_is_stationary(&meas->rad_meas_arr[i])) {
            stationary_meas->gated_meas_idx[num_meas] = (sint16_t)i;
            num_meas++;
        }
    }
    stationary_meas->num_meas_gated = num_meas;
}


void predicted_vxvy_and_compute_error(
    const estimated_ego_motion* ego_motion_pred,
    const meas* rad_meas,
    point_2d* error) {

    /* compute predicted vx vy */
    REAL vx_pred = -( ego_motion_pred->vx - rad_meas->py * ego_motion_pred->yaw_rate );
    REAL vy_pred = -( ego_motion_pred->vy + rad_meas->px * ego_motion_pred->yaw_rate );

    /* compute error */
    error->x = rad_meas->vx - vx_pred;
    error->y = rad_meas->vy - vy_pred;
}


void compute_errors(
    const estimated_ego_motion* ego_motion_pred,
    const meas_aos* meas,
    const gated_meas_ids* stationary_meas,
    errors* error) {

    uint16_t idx, i;

    for (i = 0; i < stationary_meas->num_meas_gated; i++) {
        idx = stationary_meas->gated_meas_idx[i];
        predicted_vxvy_and_compute_error(ego_motion_pred, &meas->rad_meas_arr[idx], &error->error_arr[i]);
    }
    error->num_valid_meas = stationary_meas->num_meas_gated;
}


void compute_error_statistics(
    const errors* error, 
    error_statistics* error_stat) {

    /* init variables */
    uint16_t i;
    REAL mu_x = 0.0, mu_y = 0.0;
    REAL dx = 0.0, dy = 0.0;
    REAL dxdx = 0.0, dxdy = 0.0, dydy = 0.0;

    /* compute sample mean */
    for (i = 0; i < error->num_valid_meas; i++) {
        mu_x += error->error_arr[i].x;
        mu_y += error->error_arr[i].y;
    }
    mu_x /= error->num_valid_meas;
    mu_y /= error->num_valid_meas;

    /* compute sample covariance */
    for (i = 0; i < error->num_valid_meas; i++) {
        dx = error->error_arr[i].x - mu_x;
        dy = error->error_arr[i].y - mu_y;
        dxdx += dx * dx;
        dxdy += dx * dy;
        dydy += dy * dy;
    }
    dxdx /= (error->num_valid_meas - 1);
    dxdy /= (error->num_valid_meas - 1);
    dydy /= (error->num_valid_meas - 1);

    /* update structure */
    error_stat->error_mu[0] = mu_x;
    error_stat->error_mu[1] = mu_y;
    error_stat->error_cov[0][0] = dxdx;
    error_stat->error_cov[0][1] = dxdy;
    error_stat->error_cov[1][0] = dxdy;
    error_stat->error_cov[1][1] = dydy;
}


REAL compute_mahalanobis_dist(
    const error_statistics* error_stat, 
    const point_2d* error) {

    /* compute matrix inverse of a 2x2 matrix */
    REAL matrix_inv_2x2[2][2] = { 0 };
    REAL factor = 1 / ( error_stat->error_cov[0][0] * error_stat->error_cov[1][1] - error_stat->error_cov[1][0] * error_stat->error_cov[0][1] );
    matrix_inv_2x2[0][0] =  factor * error_stat->error_cov[1][1];
    matrix_inv_2x2[0][1] = -factor * error_stat->error_cov[0][1];
    matrix_inv_2x2[1][0] = -factor * error_stat->error_cov[1][0];
    matrix_inv_2x2[1][1] =  factor * error_stat->error_cov[0][0];

    /* compute mahalanobis distance */
    REAL err_diff_x = error->x - error_stat->error_mu[0];
    REAL err_diff_y = error->y - error_stat->error_mu[1];
    REAL a1 = err_diff_x * matrix_inv_2x2[0][0] + err_diff_y * matrix_inv_2x2[1][0];
    REAL b1 = err_diff_x * matrix_inv_2x2[0][1] + err_diff_y * matrix_inv_2x2[1][1];
    return a1 * err_diff_x + b1 * err_diff_y;
}


void perform_gating(
    const errors* error, 
    const error_statistics* error_stat, 
    const gated_meas_ids* stationary_meas, 
    gated_meas_ids* gated_meas) {

    REAL maha_dist;
    uint16_t n = 0, i;

    for (i = 0; i < stationary_meas->num_meas_gated; i++) {
        maha_dist = compute_mahalanobis_dist(error_stat, &error->error_arr[i]);
        if (maha_dist <= chi_sq_const) {
            gated_meas->gated_meas_idx[n] = stationary_meas->gated_meas_idx[i];
            n++;
        }
    }
    gated_meas->num_meas_gated = n;
}


void gate_measurements(
    const estimated_ego_motion* ego_motion_pred, 
    const meas_aos* meas, 
    gated_meas_ids* gated_meas,
    BOOL* start) {

    if (*start) {

        /* identify stationary measurements */
        gated_meas_ids stationary_meas;
        identify_stationary_meas(meas, &stationary_meas);

        /* compute errors */
        errors error;
        compute_errors(ego_motion_pred, meas, &stationary_meas, &error);

        /* compute error statistics(mu and sigma) */
        error_statistics error_stat;
        compute_error_statistics(&error, &error_stat);

        /* gate measurements */
        perform_gating(&error, &error_stat, &stationary_meas, gated_meas);
    }

    else {

        identify_stationary_meas(meas, gated_meas);
        *start = TRUE;
    }
}

/* ----------------------------------------------------------------------------------------------------------
 *                 Function Definition : ego-motion computation functions
 * ---------------------------------------------------------------------------------------------------------- */

void create_system_matrix_3dof(
    REAL A[3][3],
    REAL b[3],
    REAL sigma_b[3][3],
    const gated_meas_ids* gated_meas,
    const meas* rad_meas) {

    sint16_t idx;
    uint16_t i = 0;
    REAL temp;

    for (i = 0; i < gated_meas->num_meas_gated; i++) {
        idx = gated_meas->gated_meas_idx[i];

        /* compute A and b */
        A[0][2] -= rad_meas[idx].py;
        A[1][2] += rad_meas[idx].px;
        A[2][2] += rad_meas[idx].py * rad_meas[idx].py + rad_meas[idx].px * rad_meas[idx].px;
        
        b[0] -= rad_meas[idx].vx;
        b[1] -= rad_meas[idx].vy;
        b[2] += rad_meas[idx].py * rad_meas[idx].vx - rad_meas[idx].px * rad_meas[idx].vy;

        /* compute sigma_b */
        temp = rad_meas[idx].vx_rms * rad_meas[idx].py;
        sigma_b[0][0] += rad_meas[idx].vx_rms;
        sigma_b[0][2] -= temp;
        sigma_b[2][0] -= temp;
        sigma_b[2][2] += temp * rad_meas[idx].py;

        temp = rad_meas[idx].vy_rms * rad_meas[idx].px;
        sigma_b[1][1] += rad_meas[idx].vy_rms;
        sigma_b[1][2] += temp;
        sigma_b[2][1] += temp;
        sigma_b[2][2] += temp * rad_meas[idx].px;
    }

    A[0][0] = gated_meas->num_meas_gated;
    A[1][1] = gated_meas->num_meas_gated;
    A[2][0] = A[0][2];
    A[2][1] = A[1][2];
    A[1][0] = A[0][1] = 0;

    sigma_b[0][1] = sigma_b[1][0] = 0;
}


void solve_3dof(
    estimated_ego_motion* ego_motion, 
    REAL A[3][3], 
    REAL b[3], 
    REAL sigma_b[3][3]) {

    REAL inverseA[3][3], est_vector[3];
    REAL temp1[3][3], est_cov[3][3];

    matrix_inverse(&A[0][0], 3, &inverseA[0][0]);
    matrix_vector_multiply_MV(&inverseA[0][0], 3, 3, &b[0], 3, &est_vector[0]);

    matrix_matrix_multiply_AB(&inverseA[0][0], 3, 3, &sigma_b[0][0], 3, 3, &temp1[0][0]);
    matrix_matrix_multiply_ABt(&temp1[0][0], 3, 3, &inverseA[0][0], 3, 3, &est_cov[0][0]);

    ego_motion->vx = est_vector[0];
    ego_motion->vy = est_vector[1];
    ego_motion->yaw_rate = est_vector[2];
}


void compute_ego_motion_3dof_lse(
    estimated_ego_motion* ego_motion,
    const gated_meas_ids* gated_meas,
    const meas_aos* meas) {

    REAL A[3][3] = { 0 };
    REAL b[3] = { 0 };
    REAL sigma_b[3][3] = { 0 };

    create_system_matrix_3dof(A, b, sigma_b, gated_meas, &meas->rad_meas_arr[0]);

    solve_3dof(ego_motion, A, b, sigma_b);

    ego_motion->timestamp_sec = meas->timestamp_sec;
}