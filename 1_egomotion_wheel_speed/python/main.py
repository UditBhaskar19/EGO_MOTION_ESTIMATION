import numpy as np
from matplotlib import pyplot as plt
import os, const
from read_data import load_can_signal
from plot_func import (
    plot_can_ego_motion,
    plot_input_signals
)
from ego_motion import (
    convert_meas_unit,
    compute_wheel_steer_angles,
    compute_wheel_locations,
    compute_ego_motion_lse,
    gate_wheel_speed
)



def ego_motion_lse(can_data, can_attr):

    # for visualization
    num_timesteps = can_data['zoe_veh_data'][:, can_attr['zoe_veh_fields']['timestamp_sec']].shape[0]
    wheel_speed_rpm = np.zeros((num_timesteps, 4))
    act_wheel_speeds = np.zeros((num_timesteps, 4))
    steering_angle = np.zeros((num_timesteps, ))
    lse_ego_motion = np.zeros((num_timesteps, 3))

    # pre-compute the wheel location coordinates
    wheel_locations = compute_wheel_locations()

    for t in range(num_timesteps):

        # extract wheel speeds and the steering angle
        FL_whl_spd = can_data['zoe_veh_data'][t, can_attr['zoe_veh_fields']['FL_wheel_speed']]
        FR_whl_spd = can_data['zoe_veh_data'][t, can_attr['zoe_veh_fields']['FR_wheel_speed']] 
        RR_whl_spd = can_data['zoe_veh_data'][t, can_attr['zoe_veh_fields']['RR_wheel_speed']] 
        RL_whl_spd = can_data['zoe_veh_data'][t, can_attr['zoe_veh_fields']['RL_wheel_speed']] 
        steer_angle = can_data['zoe_veh_data'][t, can_attr['zoe_veh_fields']['steering_angle']]

        # convert wheel speed from tpm to m/s and compute the wheel steer angles
        whl_spd = convert_meas_unit(FL_whl_spd, FR_whl_spd, RR_whl_spd, RL_whl_spd)
        delta = compute_wheel_steer_angles(steer_angle)

        # compute ego motion
        if t == 0:
            ego_motion, _ = compute_ego_motion_lse(whl_spd, const.noise_cov_wheels, delta, wheel_locations)
        else:
            x_pred = ego_motion
            _, wheel_idx = gate_wheel_speed(x_pred, wheel_locations, whl_spd, delta)
            ego_motion, _ = compute_ego_motion_lse(whl_spd[wheel_idx], const.noise_cov_wheels[wheel_idx], delta[wheel_idx], wheel_locations[wheel_idx])
            
        # consolidate results for plotting
        wheel_speed_rpm[t, :] = np.array([FL_whl_spd, FR_whl_spd, RR_whl_spd, RL_whl_spd])
        act_wheel_speeds[t, :] = whl_spd
        steering_angle[t] = steer_angle
        lse_ego_motion[t, :] = np.array([ego_motion[0], 0.0, ego_motion[1]])

        # display
        time = can_data['zoe_veh_data'][t, can_attr['zoe_veh_fields']['timestamp_sec']]
        print('time: ', f'{time:.3f}' , '    vx: ', f'{lse_ego_motion[t,0]:.3f}', '    yaw_rate: ', f'{(180/np.pi)*lse_ego_motion[t,2]:.3f}')

    return lse_ego_motion, wheel_speed_rpm, act_wheel_speeds, steering_angle



if __name__ == '__main__':
    scene_id = 7    # 0-8  0, 1, 2, 3, 4, 5*, 6, 7*, 8
    scene_names = os.listdir(const.root_dir)
    scene_dir = os.path.join(const.root_dir, scene_names[scene_id])
    can_data, can_attr = load_can_signal(scene_dir)

    lse_ego_motion, wheel_speed_rpm, actual_wheel_speeds, steering_angle = ego_motion_lse(can_data, can_attr)

    plot_can_ego_motion(lse_ego_motion, actual_wheel_speeds, can_data, can_attr, tag='Ordinary Least Squares', scene_id=scene_names[scene_id])
    plot_input_signals(wheel_speed_rpm, steering_angle, can_data, can_attr, scene_names[scene_id])
    plt.show()