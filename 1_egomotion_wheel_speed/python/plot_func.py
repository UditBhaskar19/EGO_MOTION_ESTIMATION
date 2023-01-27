from matplotlib import pyplot as plt
import numpy as np


def plot_input_signals(actual_wheel_speeds, steering_angle, can_data, can_attr, scene_id):

    time_zoe_veh_info = can_data['zoe_veh_data'][:, can_attr['zoe_veh_fields']['timestamp_sec']]
    fig, ax = plt.subplots(1,2)

    # wheel speeds
    ax[0].plot(time_zoe_veh_info, actual_wheel_speeds[:, 0], '-', color='red', label='Front Left wheel speed')
    ax[0].plot(time_zoe_veh_info, actual_wheel_speeds[:, 1], '-', color='green', label='Front Right wheel speed')
    ax[0].plot(time_zoe_veh_info, actual_wheel_speeds[:, 2], '-', color='blue', label='Rear Right wheel speed')
    ax[0].plot(time_zoe_veh_info, actual_wheel_speeds[:, 3], '-', color='magenta', label='Rear Left wheel speed')
    ax[0].legend()
    ax[0].set_xlabel('time (sec)')
    ax[0].set_ylabel('speed (rpm)')

    # steering angle
    ax[1].plot(time_zoe_veh_info, steering_angle, '-', color='black', label='steering angle')
    ax[1].legend()
    ax[1].set_xlabel('time (sec)')
    ax[1].set_ylabel('degree')

    # fig.tight_layout()
    fig.suptitle('Plot of input signals for ' + scene_id, fontsize=16)


def plot_can_ego_motion(est_ego_motion, actual_wheel_speeds, can_data, can_attr, tag, scene_id):

    # timestamps
    time_zoe_veh_info = can_data['zoe_veh_data'][:, can_attr['zoe_veh_fields']['timestamp_sec']]
    time_loc = can_data['pose_data'][:, can_attr['pose_fields']['timestamp_sec']]

    # vx, vy, yaw_rate from gt
    vx_gt = can_data['pose_data'][:, can_attr['pose_fields']['vx']]
    yaw_rate_gt = can_data['pose_data'][:, can_attr['pose_fields']['rot_rate_z']]

    # ------------------------------------------------------------------------------------------------------------------------

    fig, ax = plt.subplots(1,2)
    # vx
    ax[0].plot(time_zoe_veh_info, est_ego_motion[:, 0], '-', color='red', label='vx ' + tag)
    ax[0].plot(time_loc, vx_gt, '-', color='green', label='vx gt')
    ax[0].legend()
    ax[0].set_xlabel('time (sec)')
    ax[0].set_ylabel('speed (m/s)')

    # yaw rate
    ax[1].plot(time_zoe_veh_info, (180/np.pi)*est_ego_motion[:, 2], '-', color='red', label='yaw rate ' + tag)
    ax[1].plot(time_loc, (180/np.pi)*yaw_rate_gt, '-', color='green', label='yaw rate Ground Truth')
    ax[1].legend()
    ax[1].set_xlabel('time (sec)')
    ax[1].set_ylabel('yaw rate (deg/s)')
    fig.suptitle('Plot of of the ego motion estimations for  ' + scene_id, fontsize=16)
    #fig.tight_layout()

    # ------------------------------------------------------------------------------------------------------------------------

    fig, ax = plt.subplots()
    ax.plot(time_zoe_veh_info, actual_wheel_speeds[:, 0], '-', color='red', label='Front Left wheel speed')
    ax.plot(time_zoe_veh_info, actual_wheel_speeds[:, 1], '-', color='green', label='Front Right wheel speed')
    ax.plot(time_zoe_veh_info, actual_wheel_speeds[:, 2], '-', color='blue', label='Rear Right wheel speed')
    ax.plot(time_zoe_veh_info, actual_wheel_speeds[:, 3], '-', color='magenta', label='Rear Left wheel speed')
    ax.plot(time_zoe_veh_info, est_ego_motion[:,0], '-', color='black', label= 'estimated wheel speed by ' + tag)
    ax.legend()
    ax.set_xlabel('time (sec)')
    ax.set_ylabel('speed (m/s)')
    fig.suptitle('Comparison of input wheel speeds and estimated velocity for ' + scene_id, fontsize=16)
    #fig.tight_layout()

    # ------------------------------------------------------------------------------------------------------------------------

    fig, ax = plt.subplots(2,2)
    # front left wheel
    ax[0,0].plot(time_zoe_veh_info, actual_wheel_speeds[:, 0], '-', color='red', label='Front Left wheel speed')
    ax[0,0].plot(time_zoe_veh_info, est_ego_motion[:, 0], '-', color='black', label= 'estimated wheel speed')
    ax[0,0].legend()
    ax[0,0].set_xlabel('time (sec)')
    ax[0,0].set_ylabel('speed (m/s)')
    
    # front right wheel
    ax[0,1].plot(time_zoe_veh_info, actual_wheel_speeds[:, 1], '-', color='green', label='Front Right wheel speed')
    ax[0,1].plot(time_zoe_veh_info, est_ego_motion[:, 0], '-', color='black', label= 'estimated wheel speed')
    ax[0,1].legend()
    ax[0,1].set_xlabel('time (sec)')
    ax[0,1].set_ylabel('speed (m/s)')

    # front left wheel
    ax[1,0].plot(time_zoe_veh_info, actual_wheel_speeds[:, 2], '-', color='blue', label='Rear Right wheel speed')
    ax[1,0].plot(time_zoe_veh_info, est_ego_motion[:, 0], '-', color='black', label= 'estimated wheel speed')
    ax[1,0].legend()
    ax[1,0].set_xlabel('time (sec)')
    ax[1,0].set_ylabel('speed (m/s)')

    # front left wheel
    ax[1,1].plot(time_zoe_veh_info, actual_wheel_speeds[:, 3], '-', color='magenta', label='Rear Left wheel speed')
    ax[1,1].plot(time_zoe_veh_info, est_ego_motion[:, 0], '-', color='black', label= 'estimated wheel speed')
    ax[1,1].legend()
    ax[1,1].set_xlabel('time (sec)')
    ax[1,1].set_ylabel('speed (m/s)')

    #fig.tight_layout()