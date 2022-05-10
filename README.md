# [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion) branch for OpenCV 4.2.0 + Eigen 3.3.9 + Ceres 2.0.0 
**[A General Optimization-based Framework for Local Odometry Estimation with Multiple Sensors (PDF)](https://arxiv.org/pdf/1901.03638.pdf)**

## Modifications:
1. all CMakeFiles.txt: set(CMAKE_CXX_FLAGS "-std=c++14")
2. #include <opencv2/imgproc/types_c.h>
   - camera_model/src/chessboard/Chessboard.cc
3. CV_AA = cv::LINE_AA, CV_GRAY2BGR = cv::COLOR_GRAY2BGR, CV_RGB2GRAY = cv::COLOR_RGB2GRAY
   - camera_model/src/intrinsic_calib.cc
   - camera_model/src/calib/CameraCalibration.cc
   - camera_model/src/chessboard/Chessboard.cc
   - vins_estimator/src/featureTracker/feature_tracker.cpp
4. cv::CALIB_CB_ADAPTIVE_THRESH, cv::CALIB_CB_NORMALIZE_IMAGE, cv::CALIB_CB_FILTER_QUADS, cv::CALIB_CB_FAST_CHECK
   - camera_model/src/chessboard/Chessboard.cc:
5. cv::FONT_HERSHEY_SIMPLEX
   - loop_fusion/src/pose_graph.cpp
6. CV_LOAD_IMAGE_GRAYSCALE = cv::IMREAD_GRAYSCALE
   - vins_estimator/src/KITTIOdomTest.cpp
   - vins_estimator/src/KITTIGPSTest.cpp
7. modify output_path & pose_graph_save_path ("./output" & "./output/pose_graph")
   - .yaml in config folder

## 1. Prerequisites
### Ubuntu 20.04.4-LTS
* Eigen 3.3.9
* Ceres 2.0.0
* Python 3.8.10
* OpenCV 4.2.0

### [ROS1 Noetic installation](http://wiki.ros.org/noetic/Installation/Ubuntu)
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt-get update
sudo apt-get install ros-noetic-desktop-full
sudo apt-get install python3-catkin-tools python3-osrf-pycommon # ubuntu 20.04
sudo apt-get install libeigen3-dev libboost-all-dev libceres-dev
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
### [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page)
```
wget https://gitlab.com/libeigen/eigen/-/archive/3.3.9/eigen-3.3.9.tar.gz
tar zxf eigen-3.3.9.tar.gz
cd eigen-3.3.9 && mkdir build && cd build
cmake ..
make -j6
sudo make install
```

### [Ceres Solver installation](http://ceres-solver.org/installation.html)
```
sudo apt-get install cmake 
sudo apt-get install libgoogle-glog-dev libgflags-dev
sudo apt-get install libatlas-base-dev
sudo apt-get install libeigen3-dev
sudo apt-get install libsuitesparse-dev

wget http://ceres-solver.org/ceres-solver-2.0.0.tar.gz
tar zxf ceres-solver-2.0.0.tar.gz
mkdir ceres-bin
cd ceres-bin
cmake ../ceres-solver-2.0.0
make -j6
make install
```
## 2. Build on ROS
```
sudo apt install metis
source /opt/ros/noetic/setup.bash
```
```
mkdir ~/catkin_vins/src
cd ~/catkin_vins/src
git clone https://github.com/rkuo/VINS-Fusion.git
cd ../
catkin build
source ~/catkin_vins/src
source devel/setup.bash
```

## 3. Demo:
### EuRoC-MAV
**MH_01_easy** (Monocualr camera + IMU)
1. `roslaunch vins vins_rviz.launch`
2. `rosrun vins vins_node ~/catkin_vins/src/VINS-Fusion/config/euroc/euroc_mono_imu_config.yaml`
3. (optional) `rosrun loop_fusion loop_fusion_node ~/catkin_vins/src/VINS-Fusion/config/euroc/euroc_mono_imu_config.yaml`
4. `rosbag play YOUR_DATASET_FOLDER/MH_01_easy.bag`
  
![](https://github.com/rkuo2000/Robotics/blob/gh-pages/images/VINS-Fusion_MH_01_easy.png?raw=true)
![](https://github.com/rkuo2000/Robotics/blob/gh-pages/images/VINS-Fusion-EoRoC-MAV-MH_01_easy.gif?raw=true)

