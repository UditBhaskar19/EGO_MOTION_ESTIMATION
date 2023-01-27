from matplotlib import pyplot as plt


def plot_ego_motion(
    data_rad, timestamp_rad, 
    data_odom, timestamp_odom,
    idx, color, radar_id, des, y_label, ax):

    i, j = idx
    label1 = des + ' using ' + radar_id
    label2 = des + ' from wheel sensors'
    ax[i,j].plot(timestamp_rad, data_rad, '-', color=color, label=label1)
    ax[i,j].plot(timestamp_odom, data_odom, '-', color='black', label=label2)
    ax[i,j].legend()
    ax[i,j].set_xlabel('time (sec)')
    ax[i,j].set_ylabel(y_label)
    return ax


def plot_ego_motion_rad(
    data_rad1, timestamp_rad1, 
    data_rad2, timestamp_rad2,
    data_rad3, timestamp_rad3,
    data_rad4, timestamp_rad4,
    des, y_label, ax):

    label = des + ' using '
    ax.plot(timestamp_rad1, data_rad1, '-', color='red', label=label + 'radar 1')
    ax.plot(timestamp_rad2, data_rad2, '-', color='green', label=label + 'radar 2')
    ax.plot(timestamp_rad3, data_rad3, '-', color='blue', label=label + 'radar 3')
    ax.plot(timestamp_rad4, data_rad4, '-', color='magenta', label=label + 'radar 4')
    ax.legend()
    ax.set_xlabel('time (sec)')
    ax.set_ylabel(y_label)
    return ax


def compare_ego_motion_radars_odometry(est1, est2, est3, est4, scene):

    vx_ego_rad1, yawrate_ego_rad1, timestamp_rad1, vx_ego_odom1, yawrate_ego_odom1, timestamp_odom1 = est1
    vx_ego_rad2, yawrate_ego_rad2, timestamp_rad2, vx_ego_odom2, yawrate_ego_odom2, timestamp_odom2 = est2
    vx_ego_rad3, yawrate_ego_rad3, timestamp_rad3, vx_ego_odom3, yawrate_ego_odom3, timestamp_odom3 = est3
    vx_ego_rad4, yawrate_ego_rad4, timestamp_rad4, vx_ego_odom4, yawrate_ego_odom4, timestamp_odom4 = est4

    fig, ax = plt.subplots(2,2)
    ax = plot_ego_motion(vx_ego_rad1, timestamp_rad1, vx_ego_odom1, timestamp_odom1, (0,0), 'red', 'radar 1', 'ego vx', 'speed (m/s)', ax)
    ax = plot_ego_motion(vx_ego_rad2, timestamp_rad2, vx_ego_odom2, timestamp_odom2, (1,0), 'green', 'radar 2', 'ego vx', 'speed (m/s)', ax)
    ax = plot_ego_motion(vx_ego_rad3, timestamp_rad3, vx_ego_odom3, timestamp_odom3, (0,1), 'blue', 'radar 3', 'ego vx', 'speed (m/s)', ax)
    ax = plot_ego_motion(vx_ego_rad4, timestamp_rad4, vx_ego_odom4, timestamp_odom4, (1,1), 'violet', 'radar 4', 'ego vx', 'speed (m/s)', ax)
    fig.suptitle('comparison of vx estimations for scene ' + scene, fontsize=16)

    fig, ax = plt.subplots(2,2)
    ax = plot_ego_motion(yawrate_ego_rad1, timestamp_rad1, yawrate_ego_odom1, timestamp_odom1, (0,0), 'red', 'radar 1', 'ego yawrate', 'yawrate (deg/s)', ax)
    ax = plot_ego_motion(yawrate_ego_rad2, timestamp_rad2, yawrate_ego_odom2, timestamp_odom2, (1,0), 'green', 'radar 2', 'ego yawrate', 'yawrate (deg/s)', ax)
    ax = plot_ego_motion(yawrate_ego_rad3, timestamp_rad3, yawrate_ego_odom3, timestamp_odom3, (0,1), 'blue', 'radar 3', 'ego yawrate', 'yawrate (deg/s)', ax)
    ax = plot_ego_motion(yawrate_ego_rad4, timestamp_rad4, yawrate_ego_odom4, timestamp_odom4, (1,1), 'violet', 'radar 4', 'ego yawrate', 'yawrate (deg/s)', ax)
    fig.suptitle('comparison of yawrate estimations for scene ' + scene, fontsize=16)



def compare_ego_motion_radars(est1, est2, est3, est4, scene):

    vx_ego_rad1, yawrate_ego_rad1, timestamp_rad1, _, _, _ = est1
    vx_ego_rad2, yawrate_ego_rad2, timestamp_rad2, _, _, _ = est2
    vx_ego_rad3, yawrate_ego_rad3, timestamp_rad3, _, _, _ = est3
    vx_ego_rad4, yawrate_ego_rad4, timestamp_rad4, _, _, _ = est4

    fig, ax = plt.subplots(1,1)
    ax = plot_ego_motion_rad(vx_ego_rad1, timestamp_rad1,
                             vx_ego_rad2, timestamp_rad2,
                             vx_ego_rad3, timestamp_rad3,
                             vx_ego_rad4, timestamp_rad4, 
                             'ego vx', 'speed (m/s)', ax)
    fig.suptitle('comparison of ego vx estimations by ols for scene ' + scene, fontsize=16)

    fig, ax = plt.subplots(1,1)
    ax = plot_ego_motion_rad(yawrate_ego_rad1, timestamp_rad1,
                             yawrate_ego_rad2, timestamp_rad2,
                             yawrate_ego_rad3, timestamp_rad3,
                             yawrate_ego_rad4, timestamp_rad4, 
                             'ego yawrate', 'yawrate (deg/s)', ax)
    fig.suptitle('comparison of ego yawrate estimations by ols for scene ' + scene, fontsize=16)


def plot_ego_motion_comparison(
    data_kf, data_ols, timestamp_rad,
    idx, color, radar_id, des, y_label, ax):

    i, j = idx
    label1 = des + ' ols using ' + radar_id
    label2 = des + ' kf using ' + radar_id
    ax[i,j].plot(timestamp_rad, data_ols, '-', color=color, label=label1)
    ax[i,j].plot(timestamp_rad, data_kf, '-', color='black', label=label2)
    ax[i,j].legend()
    ax[i,j].set_xlabel('time (sec)')
    ax[i,j].set_ylabel(y_label)
    return ax



def compare_kf_and_ols(kf1, kf2, kf3, kf4, ols1, ols2, ols3, ols4, scene):

    vx_kf1, yawrate_kf1, timestamp_rad1, _, _, _ = kf1
    vx_kf2, yawrate_kf2, timestamp_rad2, _, _, _ = kf2
    vx_kf3, yawrate_kf3, timestamp_rad3, _, _, _ = kf3
    vx_kf4, yawrate_kf4, timestamp_rad4, _, _, _ = kf4

    vx_ols1, yawrate_ols1, _, _, _, _ = ols1
    vx_ols2, yawrate_ols2, _, _, _, _ = ols2
    vx_ols3, yawrate_ols3, _, _, _, _ = ols3
    vx_ols4, yawrate_ols4, _, _, _, _ = ols4

    fig, ax = plt.subplots(2,2)
    ax = plot_ego_motion_comparison(vx_kf1, vx_ols1, timestamp_rad1, (0,0), 'tomato', 'radar 1', 'ego vx', 'speed (m/s)', ax)
    ax = plot_ego_motion_comparison(vx_kf2, vx_ols2, timestamp_rad2, (1,0), 'limegreen', 'radar 2', 'ego vx', 'speed (m/s)', ax)
    ax = plot_ego_motion_comparison(vx_kf3, vx_ols3, timestamp_rad3, (0,1), 'dodgerblue', 'radar 3', 'ego vx', 'speed (m/s)', ax)
    ax = plot_ego_motion_comparison(vx_kf4, vx_ols4, timestamp_rad4, (1,1), 'magenta', 'radar 4', 'ego vx', 'speed (m/s)', ax)
    fig.suptitle('comparison of vx estimations by kf and ols for scene ' + scene, fontsize=16)

    fig, ax = plt.subplots(2,2)
    ax = plot_ego_motion_comparison(yawrate_kf1, yawrate_ols1, timestamp_rad1, (0,0), 'tomato', 'radar 1', 'ego yawrate', 'yawrate (deg/s)', ax)
    ax = plot_ego_motion_comparison(yawrate_kf2, yawrate_ols2, timestamp_rad2, (1,0), 'limegreen', 'radar 2', 'ego yawrate', 'yawrate (deg/s)', ax)
    ax = plot_ego_motion_comparison(yawrate_kf3, yawrate_ols3, timestamp_rad3, (0,1), 'dodgerblue', 'radar 3', 'ego yawrate', 'yawrate (deg/s)', ax)
    ax = plot_ego_motion_comparison(yawrate_kf4, yawrate_ols4, timestamp_rad4, (1,1), 'magenta', 'radar 4', 'ego yawrate', 'yawrate (deg/s)', ax)
    fig.suptitle('comparison of yawrate estimations by kf and ols for scene ' + scene, fontsize=16)