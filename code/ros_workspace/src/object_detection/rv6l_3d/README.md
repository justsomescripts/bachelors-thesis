# Package for object detection and localization using Darknet

Modified package for RV6L. Detects coordinate of Bounding Box center and distance between camera and object.

Source: [https://github.com/IntelligentRoboticsLabs/gb_visual_detection_3d](https://github.com/IntelligentRoboticsLabs/gb_visual_detection_3d) (Original description below)  

***

### Contents
* Overview
  * Previous steps
* Quick start
  * Instalation
  * How it works
* Nodes
  * darknet3d_node

### Dependencies

* roscpp
* darknet_ros_msgs
* [rv6l_3d_msgs](/ros_workspace/src/rv6l_3d_msgs)
* sensor_msgs
* tf
* pcl_ros
* pcl_conversions
* roslint

**You can install Darknet ROS following** [those steps](https://github.com/leggedrobotics/darknet_ros)

### Configuration parameters

* **interested_classes:** Classes you want to detect. It must be classes names than exists previously in darknet ros.

* **mininum_detection_th***e***reshold:** Maximum distance range between any pixel of image and the center pixel of the image to be considered.

* **minimum_probability:** Minimum object probability (provided by *darknet_ros*) to be considered.

* **darknet_ros_topic:** topic where darknet ros publicates his bounding boxes. ``/darknet_ros/bounding_boxes``

* **point_cloud_topic:** topic where point cloud is published from camera. By default: ``/camera/depth_registered/points``. **It is important that point cloud topic be of PointCloud2 type and it be depth_registered**

* **working_frame:** frame that all measurements are based on. By default, *camera_link*.

**NOTE:** color image topic that darknet_ros needs, can be edited in the launch file retyping *camera_rgb_topic* argument.

## Nodes

### darknet3d_node

*darknet3d_node* provide bounding boxes. This bounding boxes are combinated with point cloud information to calculate (xmin, ymin, zmin) and (xmax, ymax, zmax) 3D coordinates.

Then, *darknet_ros_3d* publicates his own bounding boxes array of *BoundingBoxes3d* type, which is an array of *BoundingBox3d* that contains the following information:
```
string Class
float32 probability
float32 w
float32 h
float32 d
```
* **Class:** Object name.
* **probability:** Probability of certainty.
* **w:** Horizontal (on image) coordinate of object center.
* **h:** Vertical (on image) coordinate of object center.
* **d:** Distance between camera and Object surface.

**Example:**

```
bounding_boxes:
  -
    Class: "person"
    probability: 0.805726051331
    w: 0.1600689888
    h: -0.267071247101
    d: 0.186251342297
```
