# Vehicle Navigation System Using GPS and IMU
![image](https://github.com/user-attachments/assets/d4ce7361-508d-457d-aa31-8189d526ac9e)

## Overview

This repository contains a comprehensive system for vehicle navigation utilizing GPS and IMU sensors. The goal is to integrate data from these sensors to improve vehicle tracking and navigation capabilities, while also analyzing the relative strengths and drawbacks of each sensor type.

## Project Components

### Hardware

- **GNSS Puck**: USB-based GPS receiver.
- **Vectornav VN-100 IMU**: USB-based Inertial Measurement Unit.

### Data Collection

To collect data for this project:
1. **Mounting Sensors**:
   - Attached the GPS puck to the roof of the vehicle.
   - Mounted the IMU inside the vehicle with its x-axis pointing forward and as horizontal as possible.
2. **Data Logging**:
   - Started logging data from both sensors before initiating the vehicle movement.
   - Drove a route that includes several 360-degree turns for compass calibration, then return to the starting point.

### Analysis and Implementation

#### 1. Heading Estimation

- **Magnetometer Calibration**: Adjust for "hard-iron" and "soft-iron" effects using circular calibration data.
- **Yaw Angle Calculation**: 
  - Calculated yaw angle from corrected magnetometer readings.
  - Integrated yaw rate from the gyroscope to obtain the yaw angle.
  - Used a complementary filter to combine both estimates for improved accuracy.

#### 2. Forward Velocity Estimation

- **Integration**: Estimated forward velocity by integrating acceleration data.
- **Comparison**: Compared this with GPS-derived velocity estimates and adjust for accuracy.

#### 3. Dead Reckoning with IMU

- **Displacement Calculation**:
  - Integrated IMU data to compute displacement and compare it with GPS data.
  - Used heading data to rotate acceleration vectors into a fixed reference frame and estimate vehicle trajectory.
  - Compared the estimated trajectory with the GPS track.

### Installation

1. Clone this repository:
   ```bash
   git clone https://github.com/arvinderss9299/Navigation-using-IMU-GPS
