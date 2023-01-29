# =====================================================================================================================
# Author Name : Udit Bhaskar
# description : library functions
# =====================================================================================================================

import numpy as np
import const


def convert_meas_unit(FL_whl_spd, FR_whl_spd, RR_whl_spd, RL_whl_spd):
    """ convert speed from rpm to m/s
    Input : FL_whl_spd - Front Left wheel rotation speed in rpm
            FR_whl_spd - Front Right wheel rotation speed in rpm
            RL_whl_spd - Rear Left wheel rotation speed in rpm
            RR_whl_spd - Rear Right wheel rotation speed in rpm
            steer_angle - steering angle in degrees
    Output : FL_whl_spd - Front Left wheel rotation speed in m/s
             FR_whl_spd - Front Right wheel rotation speed in m/s
             RL_whl_spd - Rear Left wheel rotation speed in m/s
             RR_whl_spd - Rear Right wheel rotation speed in m/s
             steer_angle - steering angle in radians
    """
    whl_spd = np.zeros([4, ])
    circumference = 2 * np.pi * const.zoe_spec['wheel_radius']
    whl_spd[0] = FL_whl_spd * circumference / 60
    whl_spd[1] = FR_whl_spd * circumference / 60
    whl_spd[2] = RR_whl_spd * circumference / 60
    whl_spd[3] = RL_whl_spd * circumference / 60
    return whl_spd


def compute_wheel_steer_angles(steer_angle):
    """ Compute wheel steer angles from steering angles
    Input : steer_angle - steering angle in radians
    Output : deltaFL - Front Left wheel steer angle in radians
             deltaFR - Front Right wheel steer angle in radians
             deltaRR - Rear Left wheel steer angle in radians
             deltaRL - Rear Right wheel steer angle in radians
    """
    delta = np.zeros([4, ])
    whl_steer_angle = (np.pi/180) * steer_angle/const.zoe_spec['steering_ratio']
    cot_whl_steer_angle = 1/np.tan(whl_steer_angle)
    term = const.zoe_spec['track']/(2*const.zoe_spec['wheelbase'])
    delta[0] = np.arctan(1/(cot_whl_steer_angle + term))
    delta[1] = np.arctan(1/(cot_whl_steer_angle - term))
    delta[2] = 0.0
    delta[3] = 0.0
    return delta


def compute_wheel_locations():
    """ Compute wheel coordinates given the wheel base and track length
    Output : wheel_locations - wheel locations
             term - a constant factor (T/2W) which shall be used to compute wheel steer angles from steering angle
             deltaRR - Rear Left wheel steer angle in radians
             deltaRL - Rear Right wheel steer angle in radians
    """
    wheel_locations =  np.zeros((4, 2))
    wheel_locations[0, :] = np.array([const.zoe_spec['wheelbase'],  const.zoe_spec['track']/2]) # FL
    wheel_locations[1, :] = np.array([const.zoe_spec['wheelbase'], -const.zoe_spec['track']/2]) # FR
    wheel_locations[2, :] = np.array([0, -const.zoe_spec['track']/2]) # RR
    wheel_locations[3, :] = np.array([0,  const.zoe_spec['track']/2]) # RL
    return wheel_locations


def gate_wheel_speed(ego_motion, wheel_locations, actual_wheel_speeds, wheel_steer_angles):
    """ Gate wheel speed given the predicted ego motion 
    Input: ego_motion - predicted ego motion vector - (vx, yaw_rate)
         : wheel_locations - wheel location coordinates
         : actual_wheel_speeds - a vector of wheel speeds
    Output: predicted_wheel_speeds - predicted wheel speed
          : wheel_idx - gated wheels
    """
    vx = ego_motion[0]
    vy = 0.0
    yaw_rate = ego_motion[1]
    num_wheels = wheel_locations.shape[0]
    predicted_wheel_speeds = np.zeros((num_wheels, ))
    for i in range(num_wheels):
        vx_pred = vx - yaw_rate*wheel_locations[i, 1]
        vy_pred = vy + yaw_rate*wheel_locations[i, 0]
        predicted_wheel_speeds[i] = vx_pred * np.cos(wheel_steer_angles[i]) + vy_pred * np.sin(wheel_steer_angles[i])
    err = np.abs(predicted_wheel_speeds - actual_wheel_speeds)
    wheel_idx = (err < const.gate_threshold).nonzero()[0]
    return predicted_wheel_speeds, wheel_idx


def compute_ego_motion_lse(whl_spd, R, delta, wheel_locations):
    """ estimate ego motion based on wheel speed, wheel steer angles
    Input : whl_spd - a vector of wheel speeds
          : R - wheel speed noise covariance
          : delta - a vector of wheel steer angles 
          : wheel_locations - wheel locations
    Output : ego_motion - estimated ego motion - (vx, yaw_rate)
           : cov - ego motion estimatio covariance of shape (2 x 2)
    """
    num_rows = 2*whl_spd.shape[0]
    A = np.zeros((num_rows, 2))
    b = np.zeros((num_rows, 1))
    B = np.zeros((num_rows, num_rows))
    sigma_b = np.zeros((num_rows, num_rows))

    for i in range(whl_spd.shape[0]):
        A[2*i,     0] = 1.0
        A[2*i + 1, 0] = 0.0

        A[2*i,     1] = -wheel_locations[i,1]
        A[2*i + 1, 1] =  wheel_locations[i,0]

        b[2*i,     0] = whl_spd[i] *  np.cos(delta[i])
        b[2*i + 1, 0] = whl_spd[i] *  np.sin(delta[i])

        B[2*i,         2*i] = np.cos(delta[i])
        B[2*i + 1, 2*i + 1] = np.sin(delta[i])
        sigma_b[2*i:(2*i + 2), 2*i:(2*i + 2)] = np.array([[R[i], R[i]], [R[i], R[i]]])

    # perform least squares
    A = np.linalg.inv( A.transpose() @ A ) @ A.transpose()

    # compute estimates
    x = A @ b
    H = A @ B
    cov = H @ sigma_b @ H.transpose()
    
    # output
    vx = x[0,0] 
    yaw_rate = x[1,0]
    ego_motion = np.array([vx, yaw_rate])
    return ego_motion, cov