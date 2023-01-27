import numpy as np

# Root directory
root_dir = '../sensor_data'

# for gating measurements
sig = 1.3
chi_sq = sig ** 2

# degree of freedom for eg-motion estimation
mode = '2dof'
# mode = '3dof'

# constants
rad2deg = 180/np.pi
deg2rad = np.pi/180

# Radar measurement attributes
"""
x,y : position of a radar cluster in radar frame in m
vx,vy : lateral and longitudinal component of the radial velocity in radar frame in m/s
vx_comp, vy_comp : ego motion compensated vx, vy in m/s
x_rms, y_rms, vx_rms, vy_rms : standard deviation
rcs : cluster rcs in dBsm
pdh0 : False alarm probability of cluster (i.e. probability of being an artefact caused by multipath or similar)
dyn_prop : Dynamic property of cluster to indicate if is moving or not
ambig_state : State of Doppler (radial velocity) ambiguity solution
valid_state : state of Cluster validity state
"""

radar_attr = [
    'x', 'y',' vx', 'vy', 'vx_comp', 'vy_comp',
    'x_rms', 'y_rms', 'vx_rms', 'vy_rms',
    'rcs', 'pdh0', 'dyn_prop', 'ambig_state', 'valid_state'
]
radar_attr = {radar_attr[i]:i for i in range(len(radar_attr))}

radar_calib_attr  = [ 'Tx', 'Ty', 'Tz', 'q1', 'q2', 'q3', 'q4']

radar_calib_attr = {radar_calib_attr[i]:i for i in range(len(radar_calib_attr))}

radar_location_attr = ['front', 'left', 'rear_left', 'rear_right', 'right']


"""
pdh0 :
0x0 (0) : invalid       0x4 (4) : <90%
0x1 (1) : <25%          0x5 (5) : <99%
0x2 (2) : <50%          0x6 (6) : <99.9%
0x3 (3) : <75%          0x7 (7) : <=100%
"""
INVALID = 999999.0
false_alarm_probability_vals = [-INVALID, 25, 50, 75, 90, 99, 99.9, 100.0]


"""
dyn_prop :
0x0 (0): moving                   0x4 (4): unknown
0x1 (1): stationary               0x5 (5): crossing stationary
0x2 (2): oncoming                 0x6 (6): crossing moving
0x3 (3): stationary candidate     0x7 (7): stopped
"""
STATIONARY = [1,3,5];  MOVING = [0,2,6]

"""
valid and invalid_state :
Invalid states
0x01 (1) invalid due to low RCS                                              Valid states
0x02 (2) invalid due to near-field artefact                                  0x00 (0) valid 
0x03 (3) invalid far range cluster because not confirmed in near range       0x04 (4) valid cluster with low RCS
0x05 (5) reserved                                                            0x08 (8) valid cluster with azimuth correction due to elevation
0x06 (6) invalid cluster due to high mirror probability                      0x09 (9) valid cluster with high child probability
0x07 (7) Invalid cluster because outside sensor field of view                0x0a (10) valid cluster with high probability of being a 50 deg artefact
0x0d (13) reserved                                                           0x0b (11) valid cluster but no local maximum
0x0e (14) invalid cluster because it is a harmonics                          0x0c (12) valid cluster with high artefact probability
                                                                             0x0f (15) valid cluster with above 95m in near range
                                                                             0x10 (16) valid cluster with high multi-target probability
                                                                             0x11 (17) valid cluster with suspicious angle
"""
"""
ambig_state :                                                                               
0x0 (0) invalid
0x1 (1) ambiguous
0x2 (2) staggered ramp
0x3 (3) unambiguous
0x4 (4) stationary candidates      
dynamic data select 1) valid states : valid(0) 
                    2) ambig_state : unambiguous (3) 
                    3) dyn_prop : moving (0, 2, 6)
"""