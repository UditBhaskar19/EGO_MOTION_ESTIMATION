# =====================================================================================================================
# Author Name : Udit Bhaskar
# description : main script
# =====================================================================================================================

from matplotlib import pyplot as plt
import numpy as np
import os, const
from read_data import (
    parse_csv_file, count_csv_files,
    select_radar_mount_parameters
)
from filter_functions import ego_motion_ols
from plot_functions import (
    compare_ego_motion_radars_odometry,
    compare_ego_motion_radars
)

# =========================================================================================================================

def ego_motion_single_radar_ols(scene, sensor, verbose=True):

    # extract odometry data and timestamps
    data_dir = os.path.join(const.root_dir, scene, sensor)
    file_timestamp_and_odom = os.path.join(data_dir, 'timestamp_and_odom.csv')
    _, data_timestamp_and_odom = parse_csv_file(file_timestamp_and_odom)

    # extract radar mount parameters
    radar_mount_param = select_radar_mount_parameters(sensor)

    # for plotting
    vx_ego_rad = []
    yawrate_ego_rad = []

    num_frames = count_csv_files(data_dir) - 1
    #num_frames = 100
    for frameid in range(num_frames):

        # extract the radar frame data
        file_meas = os.path.join(data_dir, str(frameid + 1) + '.csv')
        _, rad_meas = parse_csv_file(file_meas)

        # extract time stamps and odom
        timestamp_rad = data_timestamp_and_odom[frameid, const.odom_attr['timestamp_rad']]
        vx_odom = data_timestamp_and_odom[frameid, const.odom_attr['vx_odom']]
        yawrate_odom = data_timestamp_and_odom[frameid, const.odom_attr['yawrate_odom']]

        # ego motion estimation by ordinary least squares
        vx_ego, yawrate_ego = ego_motion_ols(rad_meas, 
                                             radar_mount_param,
                                             vx_odom, 
                                             yawrate_odom)

        # units: time->sec, vx->m/s, yawrate->deg/s
        if verbose:
            print('-' * 110)
            print('time: ', f'{timestamp_rad:.3f}' ,
                    '     vx_est: ', f'{vx_ego:.3f}' ,
                    '     yawrate_est: ', f'{const.rad2deg * yawrate_ego:.3f}' ,
                    '     vx_odom: ', f'{vx_odom:.3f}' ,
                    '     yawrate_odom: ', f'{const.rad2deg * yawrate_odom:.3f}' ,)

        vx_ego_rad.append(vx_ego)
        yawrate_ego_rad.append(const.rad2deg * yawrate_ego)

    # list to numpy arrays
    vx_ego_rad = np.array(vx_ego_rad)
    yawrate_ego_rad = np.array(yawrate_ego_rad)

    # extract ground truth localization info
    x_loc = data_timestamp_and_odom[:num_frames, const.odom_attr['x_loc']]
    y_loc = data_timestamp_and_odom[:num_frames, const.odom_attr['y_loc']]
    yaw_loc = data_timestamp_and_odom[:num_frames, const.odom_attr['yaw_loc']]

    # extract ego-motion from odometry for comparizon with the estimated ego-motion
    vx_ego_odom = data_timestamp_and_odom[:num_frames, const.odom_attr['vx_odom']]
    yawrate_ego_odom = const.rad2deg * data_timestamp_and_odom[:num_frames, const.odom_attr['yawrate_odom']]
    timestamp_odom = data_timestamp_and_odom[:num_frames, const.odom_attr['timestamp_odom']]
    timestamp_rad = data_timestamp_and_odom[:num_frames, const.odom_attr['timestamp_rad']]

    return ( 
        vx_ego_rad,
        yawrate_ego_rad,
        timestamp_rad,
        vx_ego_odom,
        yawrate_ego_odom,
        timestamp_odom),    \
        (
            x_loc, y_loc, yaw_loc
        )
    
# =========================================================================================================================

if __name__ == '__main__':

    scene = '105'  # 105, 108 
    mode = 'plots'  # 'output', 'plots'

    if mode == 'output':
        radarid = 'radar1'   # 'radar1', 'radar2', 'radar3', 'radar4'
        outputs, _ = ego_motion_single_radar_ols(scene, radarid, verbose=True)

    else:
        outputs1, _ = ego_motion_single_radar_ols(scene, 'radar1', verbose=False)
        print('radar1 ols run complete !!..')
        outputs2, _ = ego_motion_single_radar_ols(scene, 'radar2', verbose=False)
        print('radar2 ols run complete !!..')
        outputs3, _ = ego_motion_single_radar_ols(scene, 'radar3', verbose=False)
        print('radar3 ols run complete !!..')
        outputs4, _ = ego_motion_single_radar_ols(scene, 'radar4', verbose=False)
        print('radar4 ols run complete !!..')

        compare_ego_motion_radars_odometry(outputs1, outputs2, outputs3, outputs4, scene)
        compare_ego_motion_radars(outputs1, outputs2, outputs3, outputs4, scene)

        plt.show()