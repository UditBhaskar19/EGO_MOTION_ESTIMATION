import numpy as np

root_dir = '../sensor_data'                      # Root directory ( sensor data csv files )

sigma_FL = 0.03                                  # std of the front left wheel speed
sigma_FR = 0.03                                  # std of the front right wheel speed
sigma_RL = 0.03                                  # std of the rear left wheel speed
sigma_RR = 0.03                                  # std of the rear right wheel speed
noise_cov_wheels = np.array([sigma_FL**2,        # wheel speed noise covariance matrix (uncorrelated)
                             sigma_FR**2, 
                             sigma_RL**2, 
                             sigma_RR**2])

gate_threshold = 0.25

zoe_spec = {                   # Renault Zoe Sensor specifications
    'wheel_radius': 0.305,     # in m
    'wheelbase': 2.588,        # in m
    'track': 1.511,            # Zoe (m)
    'steering_ratio': 15.2,    # approximation (unitless)
}

zoe_veh_info_attr = ['timestamp_sec',    # can attributes ( in the csv file zoe_veh_signal.csv)
                     'FL_wheel_speed',
                     'FR_wheel_speed',
                     'RR_wheel_speed',
                     'RL_wheel_speed',
                     'steering_angle']