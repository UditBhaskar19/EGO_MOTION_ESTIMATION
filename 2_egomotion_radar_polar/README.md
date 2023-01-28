# EGO_MOTION_ESTIMATION
[detailed design document link](https://github.com/UditBhaskar19/EGO_MOTION_ESTIMATION/blob/main/1_egomotion_wheel_speed/1_ego_motion_from_wheel_speed.pdf)

## Introduction
Here the ego-motion estimation is performed from the **wheel speeds** and **steering angle**. It is assumed that the **vehicle is car-like with 4 wheels is a 2-Wheeled Drive**.The results are validated using **NuScenes** mini dataset.

## Contents
### 1. Sensor Setup and Layout
In this project [RadarScenes](https://radar-scenes.com/) dataset is used for validating and generating results. Here the measurements are not synchronized and the sensor layout doesnot have a full 360&deg; coverage. Nonetheless the dataset is considered here because it is one of the few datasets publickly available that has raw radar point cloud measurements.
![](https://github.com/UditBhaskar19/EGO_MOTION_ESTIMATION/blob/main/2_egomotion_radar_polar/readme_artifacts/0_sensor_setups.PNG)

### 2. Inputs Considered and Required Outputs
The inputs are the radar measurements in polar coordinates. A detailed summary of all the required inputs and the outputs is as follows.
![](https://github.com/UditBhaskar19/EGO_MOTION_ESTIMATION/blob/main/2_egomotion_radar_polar/readme_artifacts/1_inputs_outputs.PNG)

### 3. Radar Scan Visualization in Ego Vehicle frame
The below animation is a brief sequence of radar frames. It can be observed that most of the range-rate is pointed radially towards the radar location. These arrows corrospond to the stationary measurements. These are infact used for estimating the radar ego-motion which is discussed in the remained of this document. The arrows NOT pointing radially corrospond to the moving or non-stationary measurements. These dynamic objects need to be removed for the ego-motion estimator to work correctly.

[Longer sequence of radar frames](https://github.com/UditBhaskar19/EGO_MOTION_ESTIMATION/blob/main/2_egomotion_radar_polar/readme_artifacts/radar_range_rate.gif)
![](https://github.com/UditBhaskar19/EGO_MOTION_ESTIMATION/blob/main/2_egomotion_radar_polar/readme_artifacts/radar_range_rate2.gif)

### 4. High Level Architecture
   - **Wheel Speed Conversion** : Convert wheel speed input signals from rpm to m/s
   - **Wheel Coordinates Computation** : Compute wheel locations w.r.t the vehicle wheel base center. Ideally these should be determined from some calibration procedure
   - **Wheel Steer Angle Computation** : Compute Front Left and Front Right wheel steer angles from raw inpute steering signal
   - **Valid Wheel Speed Selection by Gating** : Due to road conditions and other environmental effects some wheels might be prone to skidding and slipping. Hence a gating procedure is introduced here to ensure that the wheel speed signal that are most likely corrupted are ignored for further processing. 
   - **Vehicle Ego-motion estimation** : Compute ego-motion w.r.t the wheel base center. Here it is assumed that the lateral velocity component is 0 ( vy = 0 )
![](https://github.com/UditBhaskar19/EGO_MOTION_ESTIMATION/blob/main/2_egomotion_radar_polar/readme_artifacts/2_architecture.PNG)
### 4. Results , Plots and Some Observations regarding Plots ( NuScenes mini - scene 0916 )
   - **Input Signal Plot** : It can be observed that the wheel speed signals are significantly more noisy than the steering signal. Hence in this project, the wheel speeds are treated as stochastic inputs which are assumed to be uncorrelated and normally distributed. The steering angle which is almost noiseless is treated as a control parameter. Under these considerations the problem of ego-motion estimation reduces to the linear least squares problem.
![](https://github.com/UditBhaskar19/EGO_MOTION_ESTIMATION/blob/main/1_egomotion_wheel_speed/readme_artifacts/3_input_signals.PNG)
   - **Ego motion estimation output Plot** : The estimated yaw-rate seems to be more noisy than the estimated velocities. Optionally the estimations can be made smoother by Kalman Filtering.
![](https://github.com/UditBhaskar19/EGO_MOTION_ESTIMATION/blob/main/1_egomotion_wheel_speed/readme_artifacts/4_estimated_outputs.PNG)
   - **Comparing estimated velocities and the input wheel speeds** : The variation of the estimated vx w.r.t time is as expected. Since the ego-motion is estimated w.r.t the wheel base center, at each time the value would be somewhere in the middle all the wheel speed values. Additionally it can be concluded that the estmated output appears to be less noisy than the input wheel speed signals.
![](https://github.com/UditBhaskar19/EGO_MOTION_ESTIMATION/blob/main/1_egomotion_wheel_speed/readme_artifacts/5_velocity_comparisons.PNG) 
### 5. Conclusion
Overall the presented approach for ego-motion estimation looks promising and also computationally efficient.
