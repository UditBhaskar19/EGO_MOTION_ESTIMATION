# EGO_MOTION_ESTIMATION
Here the ego-motion estimation is performed from the **wheel speeds** and **steering angle**. It is assumed that the **vehicle is car-like with 4 wheels is a 2-Wheeled Drive**.The results are validated using **NuScenes** mini dataset.

## Table of Contents

#### [P1 - Detecting Lane Lines (basic)](project_1_lane_finding_basic)
 - **Summary:** Detected highway lane lines on a video stream. Used OpencV image analysis techniques to identify lines, including Hough Transforms and Canny edge detection.
 - **Keywords:** Computer Vision
 
#### [P2 - Traffic Sign Classification](project_2_traffic_sign_classifier)
 - **Summary:** Built and trained a deep neural network to classify traffic signs, using TensorFlow. Experimented with different network architectures. Performed image pre-processing and validation to guard against overfitting.
 - **Keywords:** Deep Learning, TensorFlow, Computer Vision


## CONTENTS

### 1. Inputs Considered and Required Outputs
### 2. High Level Architecture
   - **Wheel Speed Conversion** - Convert wheel speed input signals from rpm to m/s
   - **Wheel Coordinates Computation** - Compute wheel locations w.r.t the vehicle wheel base center. Ideally these should be determined from some calibration procedure
   - **Wheel Steer Angle Computation** - Compute Front Left and Front Right wheel steer angles from raw inpute steeting signal
   - **Valid Wheel Speed Selection by Gating** - Due to road conditions and other environmental effects some wheels might be prone to skidding and slipping. hence a gating procedure is introduced here to ensure that the wheel speed signal that are most likely corrupted are ignored for further processing. 
   - **Vehicle Ego-motion estimation** - Compute ego-motion w.r.t the wheel base center. Here we assume that the lateral velocity component is 0 ( vy = 0 )
### 3. Results & Plots
### 4. Some Observations regarding Plots
