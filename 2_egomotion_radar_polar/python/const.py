import numpy as np

root_dir = '../sensor_data'

# mount info in x, y, angle
radar1_mount = np.array([3.663, -0.873, -1.48418552])
radar2_mount = np.array([3.86, -0.7, -0.436185662])
radar3_mount = np.array([3.86, 0.7, 0.436])
radar4_mount = np.array([3.663, 0.873, 1.484])
radars_mount = np.stack([radar1_mount, radar2_mount, radar3_mount, radar4_mount], axis=0)
num_radars = radars_mount.shape[0]

# ransac parameters
ransac_min_num_samples = 2
ransac_error_margin = 0.15
ransac_num_iterations = 30
inlier_ratio_threshold = 0.6

# parameters for gating stationary measurements
gamma_stationary = 1.5
gamma_sq_vx_vy = 2**2

# constants
rad2deg = 180/np.pi
deg2rad = np.pi/180

# kalman filter parameters and data struct
sigma_vx = 0.2                                   
sigma_yaw_rate = 0.02 #0.06
q = np.array([sigma_vx**2, sigma_yaw_rate**2])
R = np.array([[0.1, 0],
              [0, 0.1]])

# mapping from column index to field attributes in csv files
# radar measurement
rad_meas_attr = {}
rad_meas_attr['range'] = 0
rad_meas_attr['azimuth'] = 1
rad_meas_attr['range_rate'] = 2
rad_meas_attr['rcs'] = 3

# timestamp and odom attr
odom_attr = {}
odom_attr['timestamp_rad'] = 0
odom_attr['timestamp_odom'] = 1
odom_attr['x_loc'] = 2
odom_attr['y_loc'] = 3
odom_attr['yaw_loc'] = 4
odom_attr['vx_odom'] = 5
odom_attr['yawrate_odom'] = 6