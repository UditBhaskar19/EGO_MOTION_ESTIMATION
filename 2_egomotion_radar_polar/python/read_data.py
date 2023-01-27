import const
import csv, os
import numpy as np


def parse_csv_file(file, header=True):
    data = []
    fields = []
    with open(file, 'r') as csvfile:
        csvreader = csv.reader(csvfile)      
        if header:
            fields = next(csvreader)             
        for row in csvreader:                
            data.append(row)
        data = np.array(data, dtype = np.float64)
    return fields, data


def count_csv_files(files_dir):
    num_files = 0
    for fname in os.listdir(files_dir):
        if fname.endswith('.csv'): 
            num_files += 1
    return num_files


def select_radar_mount_parameters(radar_id):
    if radar_id == 'radar1': mount = const.radars_mount[0, :]
    elif radar_id == 'radar2': mount = const.radars_mount[1, :]
    elif radar_id == 'radar3': mount = const.radars_mount[2, :]
    elif radar_id == 'radar4': mount = const.radars_mount[3, :]
    else : print('Invalid Option !! ... '); return 0, 0, 0
    tx = mount[0]
    ty = mount[1]
    azimuth = mount[2]
    return tx, ty, azimuth