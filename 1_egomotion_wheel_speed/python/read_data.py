# =====================================================================================================================
# Author Name : Udit Bhaskar
# description : utils functions
# =====================================================================================================================

import os, csv
import numpy as np
import const


def parse_csv_file(file):
    data = []
    with open(file, 'r') as csvfile:
        csvreader = csv.reader(csvfile)      
        fields = next(csvreader)             
        for row in csvreader:                
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


if __name__ == '__main__':
    scene_dir = os.path.join(const.root_dir, 'scene-0061')
    can_signal, can_signal_attr = load_can_signal(scene_dir)
    print(can_signal_attr)