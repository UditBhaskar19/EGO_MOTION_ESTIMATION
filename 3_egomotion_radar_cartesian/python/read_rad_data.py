import os, csv, const
import numpy as np


def parse_csv_file(file):
    data = []
    with open(file, 'r') as csvfile:
        csvreader = csv.reader(csvfile)      # creating a csv reader object
        fields = next(csvreader)             # extracting field names through first row
        for row in csvreader:                # extracting each data row one by one
            data.append(row)
        data = np.array(data, dtype = np.float64)
    return fields, data


def load_can_signal(scene_dir):
    
    file = os.path.join(scene_dir, 'can', 'gt_pose.csv')
    pose_fields, pose_data = parse_csv_file(file)
    pose_fields = {pose_fields[i]:i for i in range(len(pose_fields))}

    file = os.path.join(scene_dir, 'can', 'zoe_veh_signal.csv')
    zoe_veh_fields, zoe_veh_data = parse_csv_file(file)
    zoe_veh_fields = {zoe_veh_fields[i]:i for i in range(len(zoe_veh_fields))}

    file = os.path.join(scene_dir, 'can', 'imu_signal.csv')
    imu_fields, imu_data = parse_csv_file(file)
    imu_fields = {imu_fields[i]:i for i in range(len(imu_fields))}
    
    signal_attr = {
        'pose_fields': pose_fields,
        'zoe_veh_fields': zoe_veh_fields,
        'imu_fields': imu_fields
    }
    signal = {
        'pose_data': pose_data, 
        'zoe_veh_data': zoe_veh_data,
        'imu_data': imu_data,
    }
    return signal, signal_attr


def scene_meta_info(root_dir, scene_name):

    scene_dir = os.path.join(root_dir, scene_name)

    front_rad_data_dir = os.path.join(scene_dir, 'radar', const.radar_location_attr[0])
    left_rad_data_dir = os.path.join(scene_dir, 'radar', const.radar_location_attr[1])
    rear_left_rad_data_dir = os.path.join(scene_dir, 'radar', const.radar_location_attr[2])
    rear_right_rad_data_dir = os.path.join(scene_dir, 'radar', const.radar_location_attr[3])
    right_rad_data_dir = os.path.join(scene_dir, 'radar', const.radar_location_attr[4])

    front_rad_time_stamps = os.path.join(scene_dir, 'radar', const.radar_location_attr[0], 'time_stamps_sec.csv')
    left_rad_time_stamps = os.path.join(scene_dir, 'radar', const.radar_location_attr[1], 'time_stamps_sec.csv')
    rear_left_rad_time_stamps = os.path.join(scene_dir, 'radar', const.radar_location_attr[2], 'time_stamps_sec.csv')
    rear_right_rad_time_stamps = os.path.join(scene_dir, 'radar', const.radar_location_attr[3], 'time_stamps_sec.csv')
    right_rad_time_stamps = os.path.join(scene_dir, 'radar', const.radar_location_attr[4], 'time_stamps_sec.csv')

    front_rad_calib_file = os.path.join(scene_dir, 'radar', const.radar_location_attr[0] + '_calib_extrinsic.csv')
    left_rad_calib_file = os.path.join(scene_dir, 'radar', const.radar_location_attr[1] + '_calib_extrinsic.csv')
    rear_left_rad_calib_file = os.path.join(scene_dir, 'radar', const.radar_location_attr[2] + '_calib_extrinsic.csv')
    rear_right_rad_calib_file = os.path.join(scene_dir, 'radar', const.radar_location_attr[3] + '_calib_extrinsic.csv')
    right_rad_calib_file = os.path.join(scene_dir, 'radar', const.radar_location_attr[4] + '_calib_extrinsic.csv')

    rad_data_meta_info = {
        'front_rad_data_dir': front_rad_data_dir,
        'left_rad_data_dir': left_rad_data_dir,
        'rear_left_rad_data_dir': rear_left_rad_data_dir,
        'rear_right_rad_data_dir': rear_right_rad_data_dir,
        'right_rad_data_dir': right_rad_data_dir
    }

    timestamps_meta_info = {
        'front_rad_time_stamps': front_rad_time_stamps,
        'left_rad_time_stamps': left_rad_time_stamps,
        'rear_left_rad_time_stamps': rear_left_rad_time_stamps,
        'rear_right_rad_time_stamps': rear_right_rad_time_stamps,
        'right_rad_time_stamps': right_rad_time_stamps
    }

    calib_meta_info = {
        'front_rad_calib_file': front_rad_calib_file,
        'left_rad_calib_file': left_rad_calib_file,
        'rear_left_rad_calib_file': rear_left_rad_calib_file,
        'rear_right_rad_calib_file': rear_right_rad_calib_file,
        'right_rad_calib_file': right_rad_calib_file
    }

    return rad_data_meta_info, timestamps_meta_info, calib_meta_info
    

def extract_rad_timestamps_and_calibration(sensor, timestamps_meta_info, calib_meta_info):

    _, rad_timestamp = parse_csv_file(timestamps_meta_info[sensor + '_time_stamps'])
    calib_attr, rad_calib = parse_csv_file(calib_meta_info[sensor + '_calib_file'])
    rad_calib = rad_calib.reshape(-1)
    rad_timestamp = rad_timestamp.reshape(-1)

    calib_attr = {calib_attr[i]:i for i in range(len(calib_attr))}
    Q = np.array([rad_calib[calib_attr['q1']], \
                  rad_calib[calib_attr['q2']], \
                  rad_calib[calib_attr['q3']], \
                  rad_calib[calib_attr['q4']]])

    _, _, rad_mount_yaw = convert_quaternion2eular_angles(Q)
    rad_mount_x = rad_calib[calib_attr['Tx']]
    rad_mount_y = rad_calib[calib_attr['Ty']]

    return (
        rad_timestamp,
        rad_mount_x, 
        rad_mount_y, 
        rad_mount_yaw
    )


def extract_rad_meas(sensor, idx, timestamps, rad_data_meta_info):
    """ load radar measurement from csv file """
    file = os.path.join(rad_data_meta_info[sensor + '_data_dir'], str(idx + 1) + '.csv')
    rad_meas_attr, rad_data = parse_csv_file(file)
    rad_meas_attr = {rad_meas_attr[i]:i for i in range(len(rad_meas_attr))}
    rad_sel_meas = select_meas_radar(rad_data, rad_meas_attr)
    z = rad_sel_meas[:, [rad_meas_attr['x'], rad_meas_attr['y'], rad_meas_attr['vx'], rad_meas_attr['vy']]]
    z_rms = rad_sel_meas[:, [rad_meas_attr['x_rms'], rad_meas_attr['y_rms'], rad_meas_attr['vx_rms'], rad_meas_attr['vy_rms']]]
    t = timestamps[idx]
    return z, z_rms, t


def select_meas_radar(radar_meas, radar_attr):
    """ select stationary measurements """
    dyn_prop_vals = radar_meas[:, radar_attr['dyn_prop']]
    valid_meas_vals = radar_meas[:, radar_attr['valid_state']]
    condition1 = np.logical_or(              # stationary measurements only
        np.int16(dyn_prop_vals)==1 , 
        np.int16(dyn_prop_vals)==3 , 
        np.int16(dyn_prop_vals)==5
    )
    condition2 = np.int16(valid_meas_vals)==0
    condition = np.logical_and(condition1, condition2)
    condition = condition.nonzero()[0]
    return radar_meas[condition, :]


def convert_quaternion2eular_angles(Q):

    temp1 = 2 * (Q[0] * Q[1] + Q[2] * Q[3])
    temp2 = 1 - 2 * (Q[1] ** 2 + Q[2] ** 2)
    temp3 = 2 * (Q[0] * Q[2] - Q[3] * Q[1])
    temp4 = 2 * (Q[0] * Q[3] + Q[1] * Q[2])
    temp5 = 1 - 2 * (Q[2] ** 2 + Q[3] ** 2)

    roll = np.arctan2(temp1, temp2)
    pitch = np.arcsin(temp3)
    yaw = np.arctan2(temp4, temp5)

    return roll, pitch, yaw


if __name__ == '__main__':
    rad_data_meta_info, timestamps_meta_info, calib_meta_info = scene_meta_info(const.root_dir, 'scene-0061')
    print(rad_data_meta_info)
    print(timestamps_meta_info)
    print(calib_meta_info)