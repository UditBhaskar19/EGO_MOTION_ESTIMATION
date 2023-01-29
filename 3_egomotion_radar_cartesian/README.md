# Ego-motion Estimation by Radar Sensor - nuScenes dataset
[detailed design document link](https://github.com/UditBhaskar19/EGO_MOTION_ESTIMATION/blob/main/3_egomotion_radar_cartesian/1_radar_ego_motion_cartesian.pdf)


## Introduction
Here we use the filtered measurements that are in cartesian (px, py, vx, vy). It is assumed that the **vehicle is car-like and 2WD**.

## Contents

### 1. Sensor Setup and Layout
In this project [nuScenes](https://www.nuscenes.org/) dataset is used for validating and generating results. The sensors are not synchronized and the sensor layout has a full 360&deg; coverage. The dataset is considered here because it is from a comercially available radar which gives measurements in cartesian coordinates.
<br>
![](https://github.com/UditBhaskar19/EGO_MOTION_ESTIMATION/blob/main/3_egomotion_radar_cartesian/readme_artifacts/1_sensor_setup.PNG)


### 2. Inputs Considered and Required Outputs
A detailed summary of all the required inputs and the outputs are as follows.![](https://github.com/UditBhaskar19/EGO_MOTION_ESTIMATION/blob/main/3_egomotion_radar_cartesian/readme_artifacts/2_inputs_outputs.PNG)


### 3. Radar Scan Visualization in Ego Vehicle frame
The below animation is a brief sequence of radar frames. It can be observed that most of the range-rate is pointed radially towards the radar location. These arrows corrospond to the stationary measurements. These are infact used for estimating the radar ego-motion which is discussed in the remainder of this document. The arrows that points away from the sensor location or has length that appears to be of drastically different size corrosponds to measurements from dynamic objects. These dynamic measurements need to be removed for the ego-motion estimator to work correctly.

[Animation for longer sequence of radar frames](https://github.com/UditBhaskar19/EGO_MOTION_ESTIMATION/blob/main/2_egomotion_radar_polar/readme_artifacts/radar_range_rate.gif)
![](https://github.com/UditBhaskar19/EGO_MOTION_ESTIMATION/blob/main/2_egomotion_radar_polar/readme_artifacts/radar_range_rate4.gif)


### 4. High Level Architecture
Since the measurements are filtered and each measurement has information regarding its dynamic status, the architecture is relatively simplier than the architecture for [ego-motion estimation from raw radar point cloud project](https://github.com/UditBhaskar19/EGO_MOTION_ESTIMATION/tree/main/2_egomotion_radar_polar). The main components are as follows:
   - **Coordinate Transformation of measurements from sensor frame to vehicle frame** : <br> The measurements are first transformed from sensor frame to vehicle frame
   - **Stationary Measurement Identification** : <br> The measurements are compared with the predicted measurements computed from previous state estimate of the ego-motion. Only the gated measurements are considered for further processing
   - **Vehicle Ego-motion estimation** : Next the ego motion is computed w.r.t the wheel base center where it is assumed that the lateral velocity component is 0 ( vy = 0 )<br><br>
![](https://github.com/UditBhaskar19/EGO_MOTION_ESTIMATION/blob/main/3_egomotion_radar_cartesian/readme_artifacts/4_architecture.PNG)


### 6. Results & Plots
   - **estimation output plot for scene - 0061** : <br>
![](https://github.com/UditBhaskar19/EGO_MOTION_ESTIMATION/blob/main/3_egomotion_radar_cartesian/readme_artifacts/5_1_0061_all_plots.PNG)


### 7. Conclusion
Overall the presented approach for ego-motion estimation looks promising. Further details can be found in the [document](https://github.com/UditBhaskar19/EGO_MOTION_ESTIMATION/blob/main/3_egomotion_radar_cartesian/1_radar_ego_motion_cartesian.pdf)


