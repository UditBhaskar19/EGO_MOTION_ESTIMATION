import const
from matplotlib import pyplot as plt

# =======================================================================================================================================

def compare_radar_ego_motion_with_gt_combined(
    front_rad_timestamp, ego_motion_front_rad, \
    rear_left_rad_timestamp, ego_motion_rear_left_rad, \
    rear_right_rad_timestamp, ego_motion_rear_right_rad,
    can_data, can_attr, scene):
    # ----------------------------------------------------------------------------------
    # timestamps gt
    time_loc = can_data['pose_data'][:, can_attr['pose_fields']['timestamp_sec']]
    vx_gt = can_data['pose_data'][:, can_attr['pose_fields']['vx']]
    yawrate_gt = can_data['pose_data'][:, can_attr['pose_fields']['rot_rate_z']]
    # ----------------------------------------------------------------------------------
    fig, ax = plt.subplots(1,1)
    ax.plot(time_loc, vx_gt, '-', color='black', label='gt vx')
    ax.plot(front_rad_timestamp, ego_motion_front_rad[:, 0], color='red', label='front radar vx LSE')
    ax.plot(rear_left_rad_timestamp, ego_motion_rear_left_rad[:, 0], color='green', label='rear left radar vx LSE')
    ax.plot(rear_right_rad_timestamp, ego_motion_rear_right_rad[:, 0], color='blue', label='rear right radar vx LSE')
    ax.legend()
    ax.set_xlabel('time (sec)')
    ax.set_ylabel('vx (m/s)')
    fig.suptitle('comparison of vx estimations for ' + scene, fontsize=16)
    # ----------------------------------------------------------------------------------
    fig, ax = plt.subplots(1,1)
    ax.plot(time_loc, const.rad2deg*yawrate_gt, '-', color='black', label='gt yawrate')
    ax.plot(front_rad_timestamp, const.rad2deg*ego_motion_front_rad[:, 2], color='red', label='front radar yaw rate LSE')
    ax.plot(rear_left_rad_timestamp, const.rad2deg*ego_motion_rear_left_rad[:, 2], color='green', label='rear left radar yaw rate LSE')
    ax.plot(rear_right_rad_timestamp, const.rad2deg*ego_motion_rear_right_rad[:, 2], color='blue', label='rear right radar yaw rate LSE')
    ax.legend()
    ax.set_xlabel('time (sec)')
    ax.set_ylabel('yaw-rate (deg/s)')
    fig.suptitle('comparison of yaw-rate estimations for ' + scene, fontsize=16)

# =======================================================================================================================================

def plot_ego_motion_comparison(
    estimation, timestamp_rad, 
    groundtruth, timestamp_loc,
    idx, color, radar, des, y_label, 
    ax):

    i, j = idx
    label1 = des + ' est by ols using ' + radar
    label2 = 'gt ' + des
    ax[i,j].plot(timestamp_rad, estimation, '-', color=color, label=label1)
    ax[i,j].plot(timestamp_loc, groundtruth, '-', color='black', label=label2)
    ax[i,j].legend()
    ax[i,j].set_xlabel('time (sec)')
    ax[i,j].set_ylabel(y_label)
    return ax 

# =======================================================================================================================================

def compare_radar_ego_motion_with_gt(
    front_rad_timestamp, ego_motion_front_rad, \
    rear_left_rad_timestamp, ego_motion_rear_left_rad, \
    rear_right_rad_timestamp, ego_motion_rear_right_rad, \
    can_data, can_attr, scene):
    # ----------------------------------------------------------------------------------
    # timestamps gt
    time_loc = can_data['pose_data'][:, can_attr['pose_fields']['timestamp_sec']]
    vx_gt = can_data['pose_data'][:, can_attr['pose_fields']['vx']]
    yawrate_gt = can_data['pose_data'][:, can_attr['pose_fields']['rot_rate_z']]
    # ----------------------------------------------------------------------------------
    # comparison of vx 
    fig, ax = plt.subplots(2,2)
    ax = plot_ego_motion_comparison(
        ego_motion_front_rad[:, 0], 
        front_rad_timestamp, 
        vx_gt, time_loc, 
        (0,0), 'tomato', 'front radar', 'ego vx', 'vx (m/s)', ax)

    ax = plot_ego_motion_comparison(
        ego_motion_rear_left_rad[:, 0], 
        rear_left_rad_timestamp, 
        vx_gt, time_loc, 
        (0,1), 'limegreen', 'rear left radar', 'ego vx', 'vx (m/s)', ax)

    ax = plot_ego_motion_comparison(
        ego_motion_rear_right_rad[:, 0], 
        rear_right_rad_timestamp, 
        vx_gt, time_loc, 
        (1,0), 'dodgerblue', 'rear right radar', 'ego vx', 'vx (m/s)', ax)
    fig.suptitle('comparison of vx estimations with gt for ' + scene, fontsize=16)
    # ----------------------------------------------------------------------------------
    # comparison of yawrate
    fig, ax = plt.subplots(2,2)
    ax = plot_ego_motion_comparison(
        const.rad2deg*ego_motion_front_rad[:, 2], 
        front_rad_timestamp, 
        const.rad2deg*yawrate_gt, time_loc, 
        (0,0), 'red', 'front radar', 'ego yawrate', 'yaw-rate (deg/s)', ax)

    ax = plot_ego_motion_comparison(
        const.rad2deg*ego_motion_rear_left_rad[:, 2], 
        rear_left_rad_timestamp, 
        const.rad2deg*yawrate_gt, time_loc, 
        (0,1), 'green', 'rear left radar', 'ego yawrate', 'yaw-rate (deg/s)', ax)

    ax = plot_ego_motion_comparison(
        const.rad2deg*ego_motion_rear_right_rad[:, 2], 
        rear_right_rad_timestamp, 
        const.rad2deg*yawrate_gt, time_loc, 
        (1,0), 'blue', 'rear right radar', 'ego yawrate', 'yaw-rate (deg/s)', ax)
    fig.suptitle('comparison of yaw-rate estimations with gt for ' + scene, fontsize=16)