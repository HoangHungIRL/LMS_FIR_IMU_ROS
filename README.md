# IMU noise attenuation using Least Mean Square (LMS) Finite Impulse Response (FIR) filter



## Overview

The filter utilizes an LMS-FIR algorithm to reduce noise in IMU accelerometer data, and can optionally be applied to gyroscope signals.

## Requirements

Ubuntu 20.04.

ROS Noetic.

## Install the package

Use the following commands:

```
mkdir -p ~/imu_filter_ws/src
cd ~/imu_filter_ws/src
git clone https://github.com/HoangHungIRL/LMS_FIR_IMU_ROS.git

cd ~/imu_filter_ws
catkin_make
```
## Launch

Use the following commands:

```
cd ~/imu_filter_ws
source devel/setup.bash
roslaunch fir_lms_imu_filter fir_lms_imu_node.launch
```

## License

This project is licensed under the MIT License.

## Contact

For questions or support, contact Hoang Quoc Hung via email hoanghung21301580@gmail.com or GitHub or open an issue in the repository.

## Acknowledgments


Developed as part of the IRL Agricultural Autonomous Vehicle project.


Implementation inspired by:

https://github.com/berndporr/fir1/tree/master
