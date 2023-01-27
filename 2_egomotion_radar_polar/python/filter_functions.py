import numpy as np
import const

from ego_motion import (
    gate_stationary_meas,
    ransac,
    coordinate_transform_vx_vy,
    estimate_ego_vx_vy_yawrate,
    estimate_sensor_vx_vy,
    kalman_filter_pred,
    compute_meas_model,
    kalman_filter_upd,
    generate_meas_sensor_frame,
    compute_P_init,
    check_if_gated
)

# =========================================================================================================================

def ego_motion_ols(
    rad_meas,
    rad_mount_param,
    vx_odom, 
    yawrate_odom):

    """ Ego motion estimation by ordinary least squares (ols)
    Inputs: rad_meas - radar measurements ( all measurements ) 
          : rad_mount_param - radar mount parameters (px, py, yaw)
          : vx_odom - temporally closest odometry vx to the radar frame
          : yaw_rate_odom - temporally closest odometry yaw_rate to the radar frame
    Outputs: vx_ego - estimated vx ego motion by ols
           : yawrate_ego - estimated yaw rate ego motion by ols
    """

    tx, ty, theta = rad_mount_param   # sensor mount param

    # perform a preliminary selection of stationary measurements
    vy_odom = 0.0
    ego_motion_prior = np.array([vx_odom, vy_odom, yawrate_odom])
    rad_meas_stationary, _ = gate_stationary_meas(ego_motion_prior, rad_meas, tx, ty, theta)

    # identify inliers by ransac
    inliers_flag, is_valid, _ = ransac(rad_meas_stationary)

    # compute vehicle ego motion 
    if is_valid:
        # use all of the inliers to estimate the vx, vy
        meas_theta = rad_meas_stationary[inliers_flag, const.rad_meas_attr['azimuth']]
        meas_vr = rad_meas_stationary[inliers_flag, const.rad_meas_attr['range_rate']]
        vx_sensor, vy_sensor = estimate_sensor_vx_vy(meas_theta, meas_vr)
        vx_sensor, vy_sensor = coordinate_transform_vx_vy(vx_sensor, vy_sensor, theta)
        vx_ego, vy_ego, yawrate_ego = estimate_ego_vx_vy_yawrate(vx_sensor, vy_sensor, tx, ty)
    else:
        vx_ego = vx_odom
        vy_ego = vy_odom
        yawrate_ego = yawrate_odom

    return vx_ego, yawrate_ego

# =========================================================================================================================

def kalman_filter_static(
    x_prev, 
    P_prev, 
    z, 
    R, 
    prev_state_exists, 
    dt):
    """ This is mainly used as a part of cascaded kalman filtering
    Inputs: ( x_prev, P_prev ) - ego motion previous state estimates
          : ( z, R ) - noisy estimates from another kalman filter
          : prev_state_exists - 0 - indicates that the previous state is valid (used for filter initialization)
          : dt - timeinterval from (t-1) to t
    Outputs: ( x_upd, P_upd ) - ego motion updated state updates
    """
    if prev_state_exists == 0:
        x_upd = z
        P_upd = R
    else:
        x_pred, P_pred = kalman_filter_pred(x_prev, P_prev, const.q1, dt)
        H = np.eye(x_pred.shape[0])
        x_upd, P_upd = kalman_filter_upd(x_pred, P_pred, z, R, H)
    return x_upd, P_upd

# =========================================================================================================================

def kalman_filter_step(
    x_prev, 
    P_prev, 
    z, 
    is_valid,
    rad_mount_param,
    vx_odom, 
    yaw_rate_odom, 
    prev_state_exists, 
    dt):

    """ Kalman filtering step. here vehicle odometry is used if the measurements are not 'trusworthy'.
        The radar radar ego motion ( vx, vy ) is computed by ordinary least squares
    Inputs: ( x_prev, P_prev ) - ego motion previous state estimates
          : (z, is_valid ) - clutter free stationary measurements 
          : is_valid - flag indicating if the measurements can be trusted
          : rad_mount_param - radar mount parameters (px, py, yaw)
          : vx_odom - temporally closest odometry vx to the radar frame
          : yaw_rate_odom - temporally closest odometry yaw_rate to the radar frame
          : prev_state_exists - 0 - indicates that the previous state is valid (used for filter initialization)
          : dt - timeinterval from (t-1) to t
    Outputs: ( x_upd, P_upd ) - ego motion state updates
    """

    tx, ty, theta = rad_mount_param   # sensor mount param

    # state initialization
    if prev_state_exists == 0:
        if is_valid:
            meas_theta = z[:, const.rad_meas_attr['azimuth']]
            meas_vr = z[:, const.rad_meas_attr['range_rate']]
            vx_sensor, vy_sensor = estimate_sensor_vx_vy(meas_theta, meas_vr)
            vx_sensor, vy_sensor = coordinate_transform_vx_vy(vx_sensor, vy_sensor, theta)
            vx_ego, vy_ego, yawrate_ego = estimate_ego_vx_vy_yawrate(vx_sensor, vy_sensor, tx, ty)
            x_upd = np.array([[vx_ego], [yawrate_ego]])
        else:
            x_upd = np.array([[vx_odom], [yaw_rate_odom]])
        P_upd =  compute_P_init(const.R, tx, ty, theta)

    # kalman filtering
    else:
        is_gated1 = False
        x_pred, P_pred = kalman_filter_pred(x_prev, P_prev, const.q, dt) 

        if is_valid:
            meas_theta = z[:, const.rad_meas_attr['azimuth']]
            meas_vr = z[:, const.rad_meas_attr['range_rate']]
            vx1, vy1 = estimate_sensor_vx_vy(meas_theta, meas_vr)
            is_gated1 = check_if_gated(x_pred[0], x_pred[1], vx1, vy1, tx, ty, theta, const.gamma_sq_vx_vy)

        vx2, vy2 = generate_meas_sensor_frame(vx_odom, 0.0, yaw_rate_odom, tx, ty, theta)
        is_gated2 = check_if_gated(x_pred[0], x_pred[1], vx2, vy2, tx, ty, theta, const.gamma_sq_vx_vy)

        condition = ( ( not is_gated1 ) and (not is_gated2 ) )
        if condition:
            x_upd = x_pred
            P_upd = P_pred
        else:
            if is_gated1:
                z = np.array([[vx1], [vy1]])
                R = const.R
            else:
                z = np.array([[vx2], [vy2]])
                R = const.R
            H = compute_meas_model(tx, ty, theta)
            x_upd, P_upd = kalman_filter_upd(x_pred, P_pred, z, R, H)

    return x_upd, P_upd

# =========================================================================================================================

def kalman_filtering_single_sensor(
    rad_meas,
    rad_mount_param,
    odom_vx,
    odom_vy, 
    odom_yawrate,
    timestamp_rad,
    timestamp_prev,
    prev_state_exists,
    x_est, 
    P_est):

    """  Ego motion estimation from a single radar by ordinary least squares and kalman filtering
    Inputs: rad_meas - radar measurements ( all measurements )
          : rad_mount_param - radar mount parameters (px, py, yaw)
          : ( odom_vx, odom_vy, odom_yawrate ) - temporally closest odometry to the radar frame
          : timestamp_rad - time stamp of the radar frame
          : timestamp_prev - time stamp of the previous radar frame
          : prev_state_exists - 0 - indicates that the previous state is valid (used for filter initialization)
          : ( x_est, P_est ) - ego motion previous state estimates
    Outputs: ( x_est, P_est ) - ego motion state updates
    """

    # extract radar mount param
    tx = rad_mount_param[0]
    ty = rad_mount_param[1] 
    theta = rad_mount_param[2]

    # perform a preliminary selection of stationary measurements
    ego_motion_prior = np.array([odom_vx, odom_vy, odom_yawrate])
    rad_meas_stationary, _ = gate_stationary_meas(ego_motion_prior, rad_meas, tx, ty, theta)

    # identify inliers by ransac
    inliers_flag, is_valid, _ = ransac(rad_meas_stationary)
    rad_meas_stationary = rad_meas_stationary[inliers_flag]

    # compute dt
    dt = timestamp_rad - timestamp_prev
    timestamp_prev = timestamp_rad

    # kalman filter 1
    x_est, P_est = kalman_filter_step(
        x_est, P_est,
        rad_meas_stationary, is_valid,
        rad_mount_param,
        odom_vx, odom_yawrate,
        prev_state_exists, dt)

    return x_est, P_est

# =========================================================================================================================