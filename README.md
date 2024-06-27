<img src="https://raw.githubusercontent.com/alstondu/Tello-SLAM/fbfb2d10c64432fe60b3cd47ad782f9ca4442845/fig/drone-thin.svg" width="100" />

# tello_slam_gazebo
Visual SLAM with DJI Tello in Gazebo

<div align="left">
<img src="https://img.shields.io/github/license/alstondu/tello_slam_gazebo ?style=flat-square&color=5D6D7E" alt="GitHub license" />
<img src="https://img.shields.io/github/last-commit/alstondu/tello_slam_gazebo?style=flat-square&color=5D6D7E" alt="git-last-commit" />
<img src="https://img.shields.io/github/commit-activity/m/alstondu/tello_slam_gazebo?style=flat-square&color=5D6D7E" alt="GitHub commit activity" />
<img src="https://img.shields.io/github/languages/top/alstondu/tello_slam_gazebo?style=flat-square&color=5D6D7E" alt="GitHub top language" />
</div>

## 🔗 Quick Links

> - [🤝 Author](#-author)
> - [📍 Overview](#-overview)
> - [🤖 Demo](#-demo)
> - [🚀 Pre-Requisites](#-pre-requisites)
> - [⚙️ Installation](#️-installation)
> - [► Running](#-running)
> - [📄 License](#-license)

---
## 🤝 Author
Yuang Du (ucab190@ucl.ac.uk)

---
## 📍 Overview
This project is to simulate DJI Tello drone in Gazebo to perform simultaneous localization and mapping (SLAM) using ORB_SLAM3.

---
## 🤖 Demo
TO DO

---
## 🚀 Pre-Requisites
TO DO

---
## ⚙️ Installation

```
git clone --recursive git@github.com:alstondu/tello_slam_gazebo.git
cd tello_slam_gazebo
catkin build
```
---
## ► Running
### 1. Camera Calibration
Launch the world with calibration board and tello:

```
roslaunch tello_driver cam_cal.launch
```

Run keyboard teleop node in terminal 2:

```
rosrun keyboard_teleop keyboard_teleop_node.py _repeat_rate:=10.0
```
Run cam_calibration node:

```
rosrun cam_calibration cameracalibrator.py --size 7x7 --square 0.25 image:=/front_cam/camera/image camera:=/front_cam
```
Drive the drone around the board until ```X, Y, Size, Skew``` all turn green. Click on the 'CALIBRATE' button, 'Save' the parameters and exit with 'COMMIT'.

### 2. Monocular SLAM
1. Launch the system in terminal 1:

	```
	roslaunch tello_driver tello.launch
	```

2.  Keyboard Control(To Do: Joystick Control)

	Run teleop node in terminal 2:

	```
	rosrun keyboard_teleop keyboard_teleop_node.py _repeat_rate:=10.0
	```

3.  Save trajectory

	Before driving the drone, in terminal 3, record the predicted trajectory(```/orb_slam3_ros/camera_pose```) and the ground truth(```/ground_truth/state```) as rosbag:
	
	```
	rosbag record /orb_slam3_ros/camera_pose /ground_truth/state
	```

4.  EVO Evaluation

	Convert the saved rosbag to tum format:
	
	```
	evo_traj bag [bag_name] /orb_slam3_ros/camera_pose /ground_truth/state --save_as_tum
	```
	
	Change the suffix of the files from ```.tum``` to ```.txt```, plot the trajectories with(```-a``` for alignment, ```-as``` for alignment and scale):
	
	```
	evo_traj tum orb_slam3_ros_camera_pose.txt --ref ground_truth_state.txt -a -p --plot_mode=xyz
	```
	For APE(Absolute Pose Error), run:
	
	```
	evo_ape tum ground_truth_state.txt orb_slam3_ros_camera_pose.txt -vas -r full -p --plot_mode=xyz
	```

## 📄 License
LICENSE: MIT.  See [LICENSE.txt](https://github.com/alstondu/tello_slam_gazebo/blob/main/LICENSE)

DISCLAIMER:

THIS INFORMATION AND/OR SOFTWARE IS PROVIDED BY THE AUTHOR "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS INFORMATION AND/OR
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Copyright (C) 2019-2024 Dimitrios Kanoulas except where specified