# =====================================================================================================================
# Author Name : Udit Bhaskar
# description : main script
# =====================================================================================================================

import numpy as np
import os, const
from matplotlib import pyplot as plt
from read_rad_data import (
    scene_meta_info, 
    load_can_signal,
    extract_rad_timestamps_and_calibration,
    extract_rad_meas
)
from ego_motion_rad import (
    cts_meas,
    meas_model,
    compute_ego_motion_lse,
    predict_meas,
    compute_errors,
    gate_meas,
    gated_meas_model
)
from plot_functions import (
    compare_radar_ego_motion_with_gt_combined,
    compare_radar_ego_motion_with_gt
)

# =======================================================================================================================================================

def compute_ego_motion(rad_data_meta_info, timestamps_meta_info, calib_meta_info, sensor):

    # extract radar mounting parameters and measurement timestamps
    rad_timestamp, \
    rad_mount_x,   \
    rad_mount_y,   \
    rad_mount_yaw = extract_rad_timestamps_and_calibration(sensor, timestamps_meta_info, calib_meta_info)

    # for plotting
    ego_motion_consolidate = []

    for idx in range(rad_timestamp.shape[0]):

        # extract the measurements
        z, z_rms, time = extract_rad_meas(sensor, idx, rad_timestamp, rad_data_meta_info)

        # coordinate transformation of the measurement from sensor frame to vehicle frame
        z = cts_meas(z, rad_mount_x, rad_mount_y, rad_mount_yaw)

        # create the measurement model
        A = meas_model(z)

        # estimation step
        if idx == 0:
            ego_motion_curr, R = compute_ego_motion_lse(z, A, z_rms, const.mode)
        else:
            z_pred = predict_meas(ego_motion_pred, A)
            err, mu, sigma = compute_errors(z, z_pred)
            gated_meas_idx, ungated_meas_idx = gate_meas(mu, sigma, err, const.chi_sq)
            A = gated_meas_model(A, gated_meas_idx)
            ego_motion_curr, R = compute_ego_motion_lse(z[gated_meas_idx, :], A, z_rms[gated_meas_idx, :], const.mode)
            
        if const.mode == '2dof': ego_motion_curr = np.array([ego_motion_curr[0], 0.0, ego_motion_curr[1]])
        ego_motion_pred = ego_motion_curr

        # output results
        print('time: ', f'{time:.3f}' , 
              '    vx: ', f'{ego_motion_curr[0]:.3f}', 
              '    yaw_rate: ', f'{const.rad2deg*ego_motion_curr[2]:.3f}')

        # for plotting
        ego_motion_consolidate.append(ego_motion_curr)
    ego_motion_consolidate = np.stack(ego_motion_consolidate, axis=0)

    return ego_motion_consolidate, rad_timestamp

# =======================================================================================================================================================        

if __name__ == '__main__':
    scene_id = 6    # 0-6   
    scene_names = os.listdir(const.root_dir)
    scene_dir = os.path.join(const.root_dir, scene_names[scene_id])
    can_data, can_attr = load_can_signal(scene_dir)
    rad_data_meta_info, timestamps_meta_info, calib_meta_info = scene_meta_info(const.root_dir, scene_names[scene_id])

    # ==========================================================================================================================

    ego_motion_front_rad, front_rad_timestamp \
            = compute_ego_motion(rad_data_meta_info, timestamps_meta_info, calib_meta_info, sensor='front_rad')
    print('complete !!..')

    ego_motion_rear_left_rad, rear_left_rad_timestamp \
            = compute_ego_motion(rad_data_meta_info, timestamps_meta_info, calib_meta_info, sensor='rear_left_rad')
    print('complete !!..')

    ego_motion_rear_right_rad, rear_right_rad_timestamp \
            = compute_ego_motion(rad_data_meta_info, timestamps_meta_info, calib_meta_info, sensor='rear_right_rad')
    print('complete !!..')

    # ==========================================================================================================================

    compare_radar_ego_motion_with_gt(
        front_rad_timestamp, ego_motion_front_rad, \
        rear_left_rad_timestamp, ego_motion_rear_left_rad, \
        rear_right_rad_timestamp, ego_motion_rear_right_rad, \
        can_data, can_attr, scene_names[scene_id]
    )

    compare_radar_ego_motion_with_gt_combined(
        front_rad_timestamp, ego_motion_front_rad, \
        rear_left_rad_timestamp, ego_motion_rear_left_rad, \
        rear_right_rad_timestamp, ego_motion_rear_right_rad, \
        can_data, can_attr, scene_names[scene_id])

    plt.show()