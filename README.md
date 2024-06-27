<p align="center">
    <img src="https://raw.githubusercontent.com/alstondu/Tello-SLAM/fbfb2d10c64432fe60b3cd47ad782f9ca4442845/fig/drone-thin.svg" width="100" />
</p>

<p align="center">
    <h1 align="center">TELLO_SLAM_GAZEBO</h1>
</p>
<p align="center">
    <em>ORB_SLAM3 with DJI Tello in Gazebo</em>
</p>

<div align="center">
<img src="https://img.shields.io/github/license/alstondu/tello_slam_gazebo ?style=flat-square&color=5D6D7E" alt="GitHub license" />
<img src="https://img.shields.io/github/last-commit/alstondu/tello_slam_gazebo?style=flat-square&color=5D6D7E" alt="git-last-commit" />
<img src="https://img.shields.io/github/commit-activity/m/alstondu/tello_slam_gazebo?style=flat-square&color=5D6D7E" alt="GitHub commit activity" />
<img src="https://img.shields.io/github/languages/top/alstondu/tello_slam_gazebo?style=flat&color=0080ff" alt="repo-top-language">
</div>

## 🔗 Quick Links

> - [🤝 Author](#-author)
> - [📍 Overview](#-overview)
> - [👾  Demo](#-demo)
> - [🗂️ Repository Structure](#-repository-structure)
> - [🚀 Pre-Requisites](#-pre-requisites)
> - [⚙️ Installation](#️-installation)
> - [► Running](#-running)
> - [🤖 Extra Instructions](#-extra-instructions)
> - [📄 License](#-license)

---
## 🤝 Author
Yuang Du (ucab190@ucl.ac.uk)

---
## 📍 Overview
This project is to simulate DJI Tello drone in Gazebo to perform simultaneous localization and mapping (SLAM) using ORB_SLAM3.

---
## 👾 Demo

**Mono SLAM:**

> **Click to watch the video demonstration.**
[![Mono ORB_SLAM3 with DJI Tello Drone in Gazebo](https://img.youtube.com/vi/5ND-YgJewG4/maxresdefault.jpg)](https://youtu.be/5ND-YgJewG4 "Mono ORB_SLAM3 with DJI Tello Drone in Gazebo")

Ape comparison w/wo scale alignment from [evo](https://github.com/MichaelGrupp/evo)
<div align="center">
<img width="100%" src="https://github.com/alstondu/tello_slam_gazebo/blob/main/media/mono_evo.png?raw=true"></a>
</div>
  
---
## 🗂️ Repository Structure

```sh
└── tello_slam_gazebo/
    ├── src
    │   ├── [hector_gazebo](git@github.com:alstondu/hector_quadrotor.git)
    │   ├── [hector_localization](git@github.com:alstondu/hector_localization.git)
    │   ├── [hector_quadrotor](git@github.com:alstondu/hector_quadrotor.git)
    │   ├── [orb_slam3_ros](git@github.com:alstondu/orb_slam3_ros.git)
    │   └── [tello_ros_gazebo](git@github.com:alstondu/tello_ros_gazebo.git)
    ├── media
    ├── LICENSE
    └── README.md
```

---
## 🚀 Pre-Requisites

See prerequisites of each submodule.

Submodule | Employed system
------------ | -------------
Gazebo plugins support | [hector_gazebo](https://github.com/alstondu/hector_gazebo)
TF and localization support | [hector_localization](https://github.com/alstondu/hector_localization)
Quadrotor controller support | [hector_quadrotor](https://github.com/alstondu/hector_quadrotor)
Visual SLAM | [orb_slam3_ros](https://github.com/alstondu/orb_slam3_ros)
Tello model and discription | [tello_ros_gazebo](https://github.com/alstondu/tello_ros_gazebo)

To evaluate the trajectory obtained, [evo](https://github.com/MichaelGrupp/evo) is required.

---
## ⚙️ Installation

```
git clone --recursive git@github.com:alstondu/tello_slam_gazebo.git
cd tello_slam_gazebo
catkin build
```

---
## ► Running
> [!TIP]
>
> <sub>source the workspace in each terminal before entering the command.</sub>

###  Monocular SLAM

#### Launch the Main System:

> Launch the system in terminal 1:

```
roslaunch tello_driver tello.launch
```

#### Keyboard Control (To Do: Joystick Control)

> Run teleop node in terminal 2:

```
rosrun keyboard_teleop keyboard_teleop_node.py _repeat_rate:=10.0
```

#### Save trajectory

> Before driving the drone, in terminal 3, record the predicted trajectory(```/orb_slam3_ros/camera_pose```) and the ground truth(```/ground_truth/state```) as rosbag:

```
rosbag record /orb_slam3_ros/camera_pose /ground_truth/state
```

#### EVO Evaluation

> Convert the saved rosbag to tum format:

```
evo_traj bag [bag_name] /orb_slam3_ros/camera_pose /ground_truth/state --save_as_tum
```

> Change the suffix of the files from ```.tum``` to ```.txt```, plot the trajectories with(```-a``` for alignment, ```-as``` for alignment and scale):

```
evo_traj tum orb_slam3_ros_camera_pose.txt --ref ground_truth_state.txt -a -p --plot_mode=xyz
```
> For APE (Absolute Pose Error), run:

```
evo_ape tum ground_truth_state.txt orb_slam3_ros_camera_pose.txt -vas -r full -p --plot_mode=xyz
```

---
## 🤖 Extra Instructions
<details closed><summary>Camera Calibration</summary>

<br>

<div align="center">
<img width="100%" src="https://github.com/alstondu/tello_slam_gazebo/blob/main/media/cam_cal.jpg?raw=true"></a>
</div>

<br>

> Launch the world with the calibration board and the tello in terminal 1:

```
roslaunch tello_driver cam_cal.launch
```

> Run the keyboard teleop node in terminal 2:

```
roslaunch tello_driver keyboard_teleop_node.launch
```

> Run cam_calibration node:

```
rosrun cam_calibration cameracalibrator.py --size 7x7 --square 0.25 image:=/front_cam/camera/image camera:=/front_cam
```

> Drive the drone around the board until ```X, Y, Size, Skew``` all turn green. Click on the 'CALIBRATE' button, 'Save' the parameters and exit with 'COMMIT'.
</details>

---
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
