# Ego-motion Estimation from radar sensor - radar point cloud is in polar coordinate
[detailed design document link](https://github.com/UditBhaskar19/EGO_MOTION_ESTIMATION/blob/main/2_egomotion_radar_polar/1_radar_ego_motion_polar.pdf)


## Introduction
Here the ego-motion estimation is performed from **radar**. It is assumed that the **vehicle is car-like and 2WD**.


## Contents

### 1. Sensor Setup and Layout
In this project [RadarScenes](https://radar-scenes.com/) dataset is used for validating and generating results. Here the measurements are not synchronized and the sensor layout doesnot have a full 360&deg; coverage. Nonetheless the dataset is considered here because it is one of the few datasets publickly available that has raw radar point cloud measurements.
![](https://github.com/UditBhaskar19/EGO_MOTION_ESTIMATION/blob/main/2_egomotion_radar_polar/readme_artifacts/0_sensor_setups.PNG)


### 2. Inputs Considered and Required Outputs
The inputs are the radar measurements in polar coordinates. A detailed summary of all the required inputs and the outputs is as follows.
![](https://github.com/UditBhaskar19/EGO_MOTION_ESTIMATION/blob/main/2_egomotion_radar_polar/readme_artifacts/1_inputs_outputs.PNG)


### 3. Radar Scan Visualization in Ego Vehicle frame
The below animation is a brief sequence of radar frames. It can be observed that most of the range-rate is pointed radially towards the radar location. These arrows corrospond to the stationary measurements. These are infact used for estimating the radar ego-motion which is discussed in the remained of this document. The arrows NOT pointing radially corrospond to the moving or non-stationary measurements. These dynamic objects need to be removed for the ego-motion estimator to work correctly.


[Animation for longer sequence of radar frames](https://github.com/UditBhaskar19/EGO_MOTION_ESTIMATION/blob/main/2_egomotion_radar_polar/readme_artifacts/radar_range_rate.gif)
![](https://github.com/UditBhaskar19/EGO_MOTION_ESTIMATION/blob/main/2_egomotion_radar_polar/readme_artifacts/radar_range_rate4.gif)


### 4. High Level Architecture
   - **Stationary Measurement Identification** : The stationary measurements are identified. First the predicted range-rate for stationarity case at each measurement (x,y) location is computed. If the measurement range-rate and the predicted range-rate is 'close' within a certain margin, then that measurement is considered for further processing. It may happen that the wheel based ego-motion is corrupted since the wheel is prone to slipping and skidding, in such a case the estimated ego-motion in the previous time t-1 is utilized for computing the predicted range-rate.
   - **Clutter Removal by RANSAC** : After an preliminary selection of the stationary measurements, Random Sampling and Consensus (RANSAC) is used to remove clutter measurements 
   - **Radar Ego-motion Computation** : Since radar gives only range-rate ( NO orthogonal velocity component ) a full 3DOF ego motion is not possible using a single radar. Here we estimate translational radar ego-motion (vx, vy) using the method of Ordinary Least Squares.
   - **Vehicle Ego-motion estimation** : Next the ego motion is computed w.r.t the wheel base center where it is assumed that the lateral velocity component is 0 ( vy = 0 )
![](https://github.com/UditBhaskar19/EGO_MOTION_ESTIMATION/blob/main/2_egomotion_radar_polar/readme_artifacts/1_architecture1.PNG)


### 4. Results , Plots and Some Observations regarding Plots ( NuScenes mini - scene 0916 )
   - **Input Signal Plot** : It can be observed that the wheel speed signals are significantly more noisy than the steering signal. Hence in this project, the wheel speeds are treated as stochastic inputs which are assumed to be uncorrelated and normally distributed. The steering angle which is almost noiseless is treated as a control parameter. Under these considerations the problem of ego-motion estimation reduces to the linear least squares problem.
![](https://github.com/UditBhaskar19/EGO_MOTION_ESTIMATION/blob/main/1_egomotion_wheel_speed/readme_artifacts/3_input_signals.PNG)
   - **Ego motion estimation output Plot** : The estimated yaw-rate seems to be more noisy than the estimated velocities. Optionally the estimations can be made smoother by Kalman Filtering.
![](https://github.com/UditBhaskar19/EGO_MOTION_ESTIMATION/blob/main/1_egomotion_wheel_speed/readme_artifacts/4_estimated_outputs.PNG)
   - **Comparing estimated velocities and the input wheel speeds** : The variation of the estimated vx w.r.t time is as expected. Since the ego-motion is estimated w.r.t the wheel base center, at each time the value would be somewhere in the middle all the wheel speed values. Additionally it can be concluded that the estmated output appears to be less noisy than the input wheel speed signals.
![](https://github.com/UditBhaskar19/EGO_MOTION_ESTIMATION/blob/main/1_egomotion_wheel_speed/readme_artifacts/5_velocity_comparisons.PNG) 
### 5. Conclusion
Overall the presented approach for ego-motion estimation looks promising and also computationally efficient.
