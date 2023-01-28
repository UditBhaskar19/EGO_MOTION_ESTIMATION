# EGO_MOTION_ESTIMATION

## Introduction
Here the ego-motion estimation is performed from the **wheel speeds** and **steering angle**. It is assumed that the **vehicle is car-like with 4 wheels is a 2-Wheeled Drive**.The results are validated using **NuScenes** mini dataset.


## Contents
### 1. Inputs Considered and Required Outputs
### 2. High Level Architecture
   - **Wheel Speed Conversion** : Convert wheel speed input signals from rpm to m/s
   - **Wheel Coordinates Computation** : Compute wheel locations w.r.t the vehicle wheel base center. Ideally these should be determined from some calibration procedure
   - **Wheel Steer Angle Computation** : Compute Front Left and Front Right wheel steer angles from raw inpute steering signal
   - **Valid Wheel Speed Selection by Gating** : Due to road conditions and other environmental effects some wheels might be prone to skidding and slipping. Hence a gating procedure is introduced here to ensure that the wheel speed signal that are most likely corrupted are ignored for further processing. 
   - **Vehicle Ego-motion estimation** : Compute ego-motion w.r.t the wheel base center. Here it is assumed that the lateral velocity component is 0 ( vy = 0 )
### 3. Results , Plots and Some Observations regarding Plots ( NuScenes mini - scene 0916 )
   - **Input Signal Plot** : It can be observed that the wheel speed signals are significantly more noisy than the steering signal. Hence in this project, the wheel speeds are treated as a stochastic inputs which are assumed to be uncorrelated and normally distributed; and the steering angle which is almost noiseless is treated as a control parameter. Under these considerations the problem of ego-motion estimation reduces to the linear least squares problem.
   - **Ego motion estimation output Plot** : 
   - **Comparing estimated velocities and the input wheel speeds** :
