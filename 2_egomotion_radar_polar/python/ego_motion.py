import numpy as np
import const


def coordinate_transform_vx_vy(vx, vy, theta):
    """ Coordinate transformation of the velocity vector from sensor frame to vehicle frame
    Input: ( vx, vy ) - velocity vector
         : theta - sensor mount azimuth angle
    Output: ( vx_cts, vy_cts ) - coordinate transformed velocity vector
    """
    vx_cts = vx * np.cos(theta) - vy * np.sin(theta)
    vy_cts = vx * np.sin(theta) + vy * np.cos(theta)
    return vx_cts, vy_cts


def coordinate_transform_px_py(px, py, tx, ty, theta):
    """ Coordinate transformation of the position vector from sensor frame to vehicle frame
    Input: ( px, py ) - position vector
         : ( tx, ty ) - sensor mount coordinates
         : theta - sensor mount azimuth angle
    Output: ( px_cts, py_cts ) - coordinate transformed position vector
    """
    px_cts = px * np.cos(theta) - py * np.sin(theta) + tx
    py_cts = px * np.sin(theta) + py * np.cos(theta) + ty
    return px_cts, py_cts


def estimate_sensor_vx_vy(meas_theta, meas_vr):
    """ estimate radar ego-motion from stationary measurements
    Input: meas_theta - an array of measurement azimuths
         : meas_vr - an array of measurement range rate azimuths
    Output: ( vx, vy ) - radar translational ego motion
    """
    A = np.zeros((2,2))
    b = np.zeros((2,1))
    n = meas_theta.shape[0]
    for i in range(n):
        cos_t  = np.cos(meas_theta[i])
        sin_t  = np.sin(meas_theta[i])
        sin_2t = np.sin(2 * meas_theta[i])
        A[0,0] += ( cos_t ** 2 )
        A[0,1] += sin_2t
        b[0,0] -= ( cos_t * meas_vr[i] )
        b[1,0] -= ( sin_t * meas_vr[i] )
    A[0,1] = 0.5 * A[0,1]
    A[1,0] = A[0,1]
    A[1,1] = n - A[0,0]
    x = np.linalg.inv(A) @ b
    return x[0,0], x[1,0]


def estimate_ego_vx_vy_yawrate(vx, vy, tx, ty):
    """ estimate vehicle ego motion w.r.t the rear wheel base center
    Input: ( vx, vy ) - radar translational ego motion w.r.t the vehicle rear wheel base center 
         : ( tx, ty ) - radar mounting position vector
    Output: ( vx_ego, vy_ego, yawrate_ego ) - vehicle ego motion
    """
    yawrate_ego = vy / tx
    vx_ego = vx + ty * yawrate_ego
    vy_ego = 0.0
    return vx_ego, vy_ego, yawrate_ego


def generate_meas_sensor_frame(vx_ego, vy_ego, yawrate_ego, tx, ty, theta):
    """ compute velocity vector in the sensor frame at the sensor location given the ego motion
    Input: vx_ego, vy_ego, yawrate_ego - vehicle ego motion
         : ( tx, ty ) - radar mounting position vector
         : theta - radar mount azimuth angle
    Output: ( vx_sensor, vy_sensor ) - velocity vector in the sensor frame at the sensor location
    """
    vx_sensor = vx_ego - yawrate_ego * ty
    vy_sensor = vy_ego + yawrate_ego * tx
    vx_sensor, vy_sensor = coordinate_transform_vx_vy(vx_sensor, vy_sensor, -theta)
    return vx_sensor, vy_sensor


def predict_range_rate_at_meas_locations(vx_ego, vy_ego, yawrate_ego, meas_theta, tx, ty, theta):
    """ compute range rate at different locations in the radar FOV give the vehicle ego motion and sensor mount info
    Input: vx_ego, vy_ego, yawrate_ego - vehicle ego motion
         : meas_theta - an array of measurement azimuths
         : ( tx, ty ) - radar mounting position vector
         : theta - radar mount azimuth angle
    Output: vr_pred - computed range rates at the measurement locations 
    """
    vx_sensor, vy_sensor = generate_meas_sensor_frame(vx_ego, vy_ego, yawrate_ego, tx, ty, theta)
    vr_pred = -( vx_sensor * np.cos(meas_theta) + vy_sensor * np.sin(meas_theta) )
    return vr_pred


def estimate_ego_motion(z_azi, z_vr, tx, ty, theta):
    """ estimate vehicle ego motion w.r.t the rear wheel base center 
    Input: z_azi - an array of measurement azimuths
         : z_vr - an array of measurement range-rates
         : ( tx, ty ) - radar mounting position vector
         : theta - radar mount azimuth angle
    Output: ( vx_ego, vy_ego, yawrate_ego ) - vehicle ego motion
    """
    vx_sensor, vy_sensor = estimate_sensor_vx_vy(z_azi, z_vr)
    vx_sensor, vy_sensor = coordinate_transform_vx_vy(vx_sensor, vy_sensor, theta)
    vx_ego, vy_ego, yawrate_ego = estimate_ego_vx_vy_yawrate(vx_sensor, vy_sensor, tx, ty)
    return vx_ego, vy_ego, yawrate_ego


def ransac(z):
    """ use RANSAC to estimate radar ego motion from radar measurements
    Input: z - a matrix of radar measurements of size (n, 4). where each vmeasurement vector is ( range, azimuth, range_rate, rcs)
    Output: inliers_flag - an array if boolean flag indicating if the corrosponding meas is an inlier
          : is_valid - a boolean flag indicating if the measurement can be trusted
          : in_ratio - ratio of the number of inliers / total number of measurement
    """
    num_meas = z.shape[0]
    is_valid = False
    in_ratio = 0
    inliers_flag = np.zeros(z.shape[0], dtype=bool)

    if num_meas > 10:

        # ransac parameters
        num_iter = const.ransac_num_iterations
        error_margin = const.ransac_error_margin
        min_num_samples = const.ransac_min_num_samples
        in_ratio_thresh = const.inlier_ratio_threshold

        # for maintaining outputs of each ransac iteratiions
        vx = np.zeros((num_iter, ))
        vy = np.zeros((num_iter, ))
        num_inliers = np.zeros((num_iter, ))
        inlier_ratio = np.zeros((num_iter, ))
    
        # for randomly selecting measurements
        meas_idx = np.arange(z.shape[0])
        for i in range(num_iter):
        
            # randomly select measurements for the consensus set and create test set
            np.random.shuffle(meas_idx)
            z_consensus_set = z[meas_idx[:min_num_samples]]
            z_test_set = z[meas_idx[min_num_samples:]]
        
            # compute the radar ego motion using the consensus set
            meas_theta = z_consensus_set[:, const.rad_meas_attr['azimuth']]
            meas_vr = z_consensus_set[:, const.rad_meas_attr['range_rate']]
            vx_est, vy_est = estimate_sensor_vx_vy(meas_theta, meas_vr)
        
            # compute the predicted vr using the computes vx, vy
            meas_theta_test = z_test_set[:, const.rad_meas_attr['azimuth']]
            meas_vr_test = z_test_set[:, const.rad_meas_attr['range_rate']] 
            pred_vr_test = -( vx_est * np.cos(meas_theta_test) + vy_est * np.sin(meas_theta_test) )
        
            # compute the errors and select the inliers
            error = np.abs(meas_vr_test - pred_vr_test)
            inliers_flag = ( error <= error_margin )
        
            # compute the output
            vx[i] = vx_est
            vy[i] = vy_est
            num_inliers[i] = np.sum(inliers_flag)
            inlier_ratio[i] = ( num_inliers[i] + min_num_samples ) / num_meas
        
        # find the index corrorspondin to the maximum number of inliers and extract vals
        max_inlier_idx = np.argmax(num_inliers)
        vx_est = vx[max_inlier_idx]
        vy_est = vy[max_inlier_idx]
        in_ratio = inlier_ratio[max_inlier_idx]
    
        # recompute the inliers  
        meas_theta = z[:, const.rad_meas_attr['azimuth']]
        meas_vr = z[:, const.rad_meas_attr['range_rate']]
        pred_vr = -( vx_est * np.cos(meas_theta) + vy_est * np.sin(meas_theta) )
        error = np.abs(meas_vr - pred_vr)
        inliers_flag = ( error <= error_margin )

        # compute the validity flag based on the in_lier ratio
        if in_ratio >= in_ratio_thresh : is_valid = True

    return inliers_flag, is_valid, in_ratio


def gate_stationary_meas(ego_motion_prior, z, tx, ty, theta):
    """ given a prior estimate of the vehicle ego motion which can be either from vehicle odometry 
        or the predicted motion estimate from the previous state ego motion select measurements which are likeliy to be stationary
        Input: ego_motion_prior - ( vx_ego, vy_ego, yawrate_ego ) - a vector of vehicle ego motion
             : z - a matrix of radar measurements of size (n, 4). where each vmeasurement vector is ( range, azimuth, range_rate, rcs)
             : ( tx, ty ) - radar mounting position vector
             : theta - radar mount azimuth angle
        Output: z -  a vector of selected measurement
              : error - a vector of errors
    """
    vx_ego = ego_motion_prior[0]
    vy_ego = ego_motion_prior[1]
    yawrate_ego = ego_motion_prior[2]

    z_azimuth = z[:, const.rad_meas_attr['azimuth']]
    z_vr = z[:, const.rad_meas_attr['range_rate']]
    vr_pred = predict_range_rate_at_meas_locations(vx_ego, vy_ego, yawrate_ego, z_azimuth, tx, ty, theta)

    error = vr_pred - z_vr
    sel_z_flag = ( np.abs(error) <= const.gamma_stationary )
    z = z[sel_z_flag]
    error = error[sel_z_flag]
    return z, error


def kalman_filter_pred(x_prev, P_prev, q, dt):
    """ Kalman filter prediction of the ego motion
    Input: x_prev - ego motion estimation at (t-1) - (vx, yaw_rate)
         : P_prev - ego motion covariance at (t-1) of shape (2 x 2)
         : q - process noise 
         : dt - time interval
    Output : predicted states - x_pred, P_pred
    """
    Q = np.diag([dt*q[0], dt*q[1]])
    P_pred = P_prev + Q
    x_pred = x_prev
    return x_pred, P_pred


def compute_meas_model(tx, ty, theta):
    """ Create the measurement model
    Input: tx, ty, theta - sensor mount parameters
    Output: H - 2x2 measurement model
    """
    Rot = np.array([[np.cos(-theta), -np.sin(-theta)],
                    [np.sin(-theta),  np.cos(-theta)]])
    H = np.array([[1.0, -ty],
                  [0.0,  tx]])
    H = Rot @ H
    return H


def kalman_filter_upd(x_pred, P_pred, z, R, H):
    """ Kalman filter state update of the ego motion
    Input: x_pred - ego motion prediction at t - (vx, yaw_rate) of shape (2 x 1)
         : P_pred - ego motion predicted covariance at t of shape (2 x 2)
         : z - measurement - (vx, vy) in the sensor frame of shape (2 x 1)
         : R - measurement noise covariance in the sensor frame of shape (2 x 2)
    Output : updated states - x, P
    """
    S = H @ P_pred @ H.transpose() + R
    K = P_pred @ H.transpose() @ np.linalg.inv(S)
    v = z - H @ x_pred
    x = x_pred + K @ v
    P = P_pred - K @ H @ P_pred
    return x, P


def compute_P_init(R, tx, ty, theta):
    """ Compute the initial state covariance
    Input: R - 2 x 2 measurement model
         : (tx, ty, theta) - radar mount parameters
    Output: P_init - initial estimate of the covariance
    """
    Rot = np.array([[np.cos(theta), -np.sin(theta)],
                    [np.sin(theta),  np.cos(theta)]])
    H = np.array([[1.0, -ty],
                  [0.0,  tx]])
    H = H @ Rot
    P_init = H @ R @ H.transpose()
    return P_init


def check_if_gated(vx, yaw_rate, meas_vx, meas_vy, tx, ty, theta, gamma_sq):
    vx_pred, vy_pred = generate_meas_sensor_frame(vx, 0.0, yaw_rate, tx, ty, theta)
    delta_sq = ( vx_pred - meas_vx )**2 + ( vy_pred - meas_vy )**2
    if delta_sq <= gamma_sq: return True
    else: return False