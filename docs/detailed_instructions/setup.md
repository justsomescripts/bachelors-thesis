# Additional detailed instructions for easy reproducibility of the project

Instructions for setting up the project on Ubuntu 20.04 LTS.  

***Changing parameters and config files is only required if the packages from the original source are used. Packages in this repo are already adjusted.***

# Setting up the environment

The instructions are only tested on ROS Noetic.  
Follow the official [installation guide](https://wiki.ros.org/noetic/Installation/Ubuntu)

Summary of commands to run:
```
# sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
# apt install curl
$ curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
# apt update
# apt install ros-noetic-desktop-full
```
Packages available in the official ROS repository can be found [here](https://index.ros.org/packages/page/1/time/#noetic). A quick search can be done using `apt search ros-noetic` or `apt-cache search ros-noetic`.

## ROS Dependencies

The following command can be used to install all requirements for the project to work correctly:  
```
# apt install ros-noetic-moveit ros-noetic-catkin python3-catkin-tools python3-osrf-pycommon python3-wstool python3-rosdep python3-rosinstall python3-rosinstall-generator ros-noetic-opencv-apps ros-noetic-pcl-ros ros-noetic-moveit-resources-prbt-moveit-config ros-noetic-moveit-visual-tools
```

Additional dependencies for RealSense 435:

```
# apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDEist:$_ros_dist
# add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
# apt install librealsense2-dkms librealsense2-utils ros-noetic-ddynamic-reconfigure librealsense2-dev ros-noetic-openni2-launch
```

### Installing darknet_ros

**Important**: Comment out `-gencode arch=compute_30,code=sm_30` in `[catkin workspace]/src/darknet_ros/darknet_ros/CMakeLists.txt`:

```
$ sed -i 's/-gencode arch=compute_30,code=sm_30/#-gencode arch=compute_30,code=sm_30/g' [catkin workspace]/src/darknet_ros/darknet_ros/CMakeLists.txt
```

The `darknet_ros` package can be built from the catkin workspace directory:
```
$ catkin build darknet_ros -DCMAKE_BUILD_TYPE=Release # catkin_make -DCMAKE_BUILD_TYPE=Release is also possible
```
Weights have to be moved into the `[catkin workspace]/src/darknet_ros/yolo_network_config/weights/` directory.

The `ros.yaml` inside the darknet_ros directory `[catkin workspace]/src/darknet_ros/darknet_ros/config/ros.yaml` file has to be adjusted for subscribing to the image topic of the RealSense. That can be done by changing `topic:` in the `subscribers:` section to `/camera/color/image_raw` using the following command:
```
$ sed -i 's|/camera/rgb/image_raw|/camera/color/image_raw|g' [catkin workspace]/src/darknet_ros/darknet_ros/config/ros.yaml
```

### Installing darknet_ros_3d

***This project uses a modified version of the package to publish the necessary center coordinates and distance instead of 3D Bounding Boxes. It's recommended to use the `rv6l_3d` package in this repository instead!*** 

#### Required

For building `darknet_ros_3d`, the directories `gb_visual_detection_3d` and `gb_visual_detection_3d_msgs` have to be included in the workspace directory. Switch the branch for `gb_visual_detection_3d` to `noetic` and for `gb_visual_detection_3d_msgs` to melodic:
```
$ cd [catkin workspace]/src/gb_visual_detection_3d && git checkout noetic
$ cd [catkin workspace]/src/gb_visual_detection_3d_msgs && git checkout melodic
```
Messages have to be built before the package:
```
$ catkin build darknet_ros_3d_msgs
$ catkin build darknet_ros_3d
```
Finally, the config has to be adjusted for using the RealSense:
contents of `[catkin workspace]/src/gb_visual_detection_3d/darknet_ros_3d/config/darknet_3d.yaml`
```
darknet_ros_topic: /darknet_ros/bounding_boxes
output_bbx3d_topic: /darknet_ros_3d/bounding_boxes
point_cloud_topic: /camera/depth_registered/points    # change sd to desired resolution [sd/qhd/hd]
working_frame: camera_link                           # required for kinect to work
mininum_detection_thereshold: 0.1                     # threshold for bounding boxes
minimum_probability: 0.1
interested_classes: ["keyboard", "person", "laptop"]  # interested classes seperated by comma
```

### With CUDA

To use CUDA for accelerated image recognition, it has to be installed using the official [instructions](https://developer.nvidia.com/cuda-downloads?target_os=Linux&target_arch=x86_64&Distribution=Ubuntu&target_version=20.04&target_type=deb_local):

```
$ wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-ubuntu2004.pin
# mv cuda-ubuntu2004.pin /etc/apt/preferences.d/cuda-repository-pin-600
$ wget https://developer.download.nvidia.com/compute/cuda/11.6.0/local_installers/cuda-repo-ubuntu2004-11-6-local_11.6.0-510.39.01-1_amd64.deb
# dpkg -i cuda-repo-ubuntu2004-11-6-local_11.6.0-510.39.01-1_amd64.deb
# apt-key add /var/cuda-repo-ubuntu2004-11-6-local/7fa2af80.pub 
# apt-get update
# apt-get -y install cuda
$ export PATH=/usr/local/cuda-11.6/bin${PATH:+:${PATH}} # replace version number
$ echo export PATH=/usr/local/cuda-11.5/bin${PATH:+:${PATH}} >> $HOME/.bashrc # adjust if another shell or CUDA version is used
```

To install the required packages, use

```
# apt install nvidia-driver-<recommended version> # you can also use Ubuntu's "Additional Drivers" program
###?? apt install cuda cuda-nvcc-<version>
# apt install gcc-7 g++-7
```

To use the correct GCC-version for compilation, add it as "gcc" to your $PATH. The `update-alternatives` package can be used as follows:

```
# update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 7
# update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-7 7
# update-alternatives --config gcc
# update-alternatives --config g++
```

If CUDA is installed, `darknet_ros` will automatically use it.

### Installing RealSense driver

```
$ cd [catkin workspace]/src
$ git clone https://github.com/IntelRealSense/realsense-ros.git
$ cd realsense-ros/
$ git checkout `git tag | sort -V | grep -P "^2.\d+\.\d+" | tail -1`
$ cd ..
$ catkin build -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release

```
