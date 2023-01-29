# Ego-motion Estimation by Radar Sensor - RadarScenes dataset
[detailed design document link](https://github.com/UditBhaskar19/EGO_MOTION_ESTIMATION/blob/main/2_egomotion_radar_polar/1_radar_ego_motion_polar.pdf)


## Introduction
Here the ego-motion estimation is performed by **radar**. It is assumed that the **vehicle is car-like and 2WD**. Radar has several advantages since it is robust against adverse weather conditions. In the event of primary sensor failure, radars can also be used to autonomusly navigate the vehicle to a safe spot. This project describes one approach for ego-motion estimation by radars. The references used in this project are listed in the document.


## Table of Contents <a name="t0"></a>

   - [Sensor Setup and Layout](#t1)
   - [Inputs Considered and Required Outputs](#t2)
   - [Radar Scan Visualization in Ego Vehicle frame](#t3)
   - [High Level Architecture](#t4)
   - [Analysis](#t5)
   - [Results, Plots and Some Observations regarding Plots](#t6)
   - [Conclusion](#t7)

<br>

### 1. Sensor Setup and Layout <a name="t1"></a>
In this project [RadarScenes](https://radar-scenes.com/) dataset is used for validating and generating results. The measurements are not synchronized and the sensor layout doesnot have a full 360&deg; coverage. Nonetheless the dataset is considered here because it is one of the few datasets publickly available that has raw radar point cloud measurements.
<br>
![](https://github.com/UditBhaskar19/EGO_MOTION_ESTIMATION/blob/main/2_egomotion_radar_polar/readme_artifacts/0_sensor_setups.PNG)
<br>

[Back to TOC](#t0)
<br>


### 2. Inputs Considered and Required Outputs <a name="t2"></a>
The inputs are the radar measurements in polar coordinates. A detailed summary of all the required inputs and the outputs are as follows.
<br>
![](https://github.com/UditBhaskar19/EGO_MOTION_ESTIMATION/blob/main/2_egomotion_radar_polar/readme_artifacts/1_inputs_outputs.PNG)
<br>

[Back to TOC](#t0)
<br>


### 3. Radar Scan Visualization in Ego Vehicle frame <a name="t3"></a>
The below animation is a brief sequence of radar frames. It can be observed that most of the range-rate is pointed radially towards the radar location. These arrows corrospond to the stationary measurements. These are infact used for estimating the radar ego-motion which is discussed in the remainder of this document. The arrows that points away from the sensor location or has length that appears to be of drastically different size corrosponds to measurements from dynamic objects. These dynamic measurements need to be removed for the ego-motion estimator to work correctly.

[Animation for longer sequence of radar frames](https://github.com/UditBhaskar19/EGO_MOTION_ESTIMATION/blob/main/2_egomotion_radar_polar/readme_artifacts/radar_range_rate.gif)
![](https://github.com/UditBhaskar19/EGO_MOTION_ESTIMATION/blob/main/2_egomotion_radar_polar/readme_artifacts/radar_range_rate4.gif)
<br>

[Back to TOC](#t0)
<br>


### 4. High Level Architecture <a name="t4"></a>
   - **Stationary Measurement Identification** : The stationary measurements are identified. First the predicted range-rate for stationarity case at each measurement (x,y) location is computed. If the measurement range-rate and the predicted range-rate is 'close' within a certain margin, then that measurement is considered for further processing. It may happen that the wheel speed based ego-motion is corrupted since the wheel is prone to slipping and skidding, in such cases the estimated ego-motion in the previous time t-1 is utilized for computing the predicted range-rate.<br>
   - **Clutter Removal by RANSAC** : After an preliminary selection of the stationary measurements, Random Sample Consensus (RANSAC) is used to remove clutter measurements.<br>
   - **Radar Ego-motion Computation** : Since radar gives only range-rate which is the radial component of the velocity vector ( NO orthogonal velocity component ) a full 3DOF ego motion is not possible using a single radar. Here we estimate translational radar ego-motion (vx, vy) using the method of Ordinary Least Squares.<br>
   - **Vehicle Ego-motion estimation** : Next the ego motion is computed w.r.t the wheel base center where it is assumed that the lateral velocity component is 0 ( vy = 0 )<br><br>
![](https://github.com/UditBhaskar19/EGO_MOTION_ESTIMATION/blob/main/2_egomotion_radar_polar/readme_artifacts/1_architecture1.PNG)
<br>

[Back to TOC](#t0)
<br>


### 5. Analysis <a name="t5"></a>
In this section some analysis is done to highlight the importance of two modules in the architecture: **Stationary Measurement Identification** & **Clutter Removal by RANSAC**
   - First, two estimation results are compared, one with and the other without the above two mentioned modules. The plot shows that the system would result in a total failure without these two modules.<br><br>
![](https://github.com/UditBhaskar19/EGO_MOTION_ESTIMATION/blob/main/2_egomotion_radar_polar/readme_artifacts/plot4.PNG)

   - We then compare the measurement range-rates with the predicted range-rates computed from the estimated radar ego-motion (vx, vy). Here the ego-motion is computed by considering all the measurements.<br>
Basically we are computing **$vr_{pred} = -( v_x * cos(theta_{meas}) + v_y * sin(theta_{meas}) )$** and plotting **$vr_{meas}$** & **$vr_{pred}$**. <br><br>
![](https://github.com/UditBhaskar19/EGO_MOTION_ESTIMATION/blob/main/2_egomotion_radar_polar/readme_artifacts/plot_misfit.PNG)

   - Next it is shown how odometry information or some other prior ego-motion estimates can be used to select only those measurements that are most likely stationary. Since RANSAC works better if a significant portion of the data are inliers, this measurement gating step is crucial. <br><br>
![](https://github.com/UditBhaskar19/EGO_MOTION_ESTIMATION/blob/main/2_egomotion_radar_polar/readme_artifacts/plot_odom_prior.PNG)

   - Finally we plot the measurements selected by RANSAC, and as seen below, the predicted range-rate line passes through the stationary measurements.
![](https://github.com/UditBhaskar19/EGO_MOTION_ESTIMATION/blob/main/2_egomotion_radar_polar/readme_artifacts/plot_ransac.PNG)
<br>

[Back to TOC](#t0)
<br>


### 6. Results , Plots and Some Observations regarding Plots ( RadarScenes - scene 105 ) <a name="t6"></a>
   - **Ego motion estimation output Plot** : The estimated yaw-rate seems to be more noisy than the estimated vx<br>
![](https://github.com/UditBhaskar19/EGO_MOTION_ESTIMATION/blob/main/2_egomotion_radar_polar/readme_artifacts/2_plots_results.PNG)

   - **Comparing OLS and KF estimates** :<br>
![](https://github.com/UditBhaskar19/EGO_MOTION_ESTIMATION/blob/main/2_egomotion_radar_polar/readme_artifacts/1_plots_results.PNG)
<br>

[Back to TOC](#t0)
<br>


### 7. Conclusion <a name="t7"></a>
Overall the presented approach for ego-motion estimation looks promising. Further details can be found in the [document](https://github.com/UditBhaskar19/EGO_MOTION_ESTIMATION/blob/main/2_egomotion_radar_polar/1_radar_ego_motion_polar.pdf)
<br>

[Back to TOC](#t0)
<br>


