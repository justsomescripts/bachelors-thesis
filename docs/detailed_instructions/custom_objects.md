# Obtaining weights

Weights can either be downloaded from sources like [this](https://pjreddie.com/darknet/yolo/) or obtained by training YOLO for a custom object as described [here](training.md).

# Using weights in Darknet and darknet_ros(_3d)

To use custom weights in darknet_ros_3d, it's recommended to do all the steps below

## Darknet

To use custom weights in Darknet, download them to the Darknet root directory. For the detection to work, a config for YOLO (v3 is used as an example here) is necessary. An example can be found [here](../../code/yolo_samples/configuration/darknet_ros/yolo_network_config/cfg). Basic instructions for the config parameters are included [here](training.md#yolo-configuration).

Detection with a static image is started using the following command:

```
$ <darknet root>/darknet detect cfg/<custom config>.cfg <custom weights>.weights data/<custom image>.jpg
```

Real-Time Detection using a webcam for example requires a custom `.data` file to work. An example is included [here](training.md#object-configuration). It is recommended to use [AlexeyABs fork of Darknet](https://github.com/AlexeyAB/darknet) because it supports OpenCV4:

```
$ <darknet root>/darknet detector demo cfg/<custom data>.data cfg/<custom config>.cfg <custom weights>.weights
```

## darknet_ros

If the detection works in Darknet, the folloing additional steps are required for darknet_ros:

* The `[catkin workspace]/src/darknet_ros/darknet_ros/config/yolov3.yaml` has to be changed as follows:

```
yolo_model:

  config_file:
    name: yolov3_custom.cfg             # custom config file name
  weight_file:
    name: yolov3_custom_last.weights    # custom weights file name
  threshold:
    value: 0.3                          # don't detect objects under a certain percentage
  detection_classes:
    names:
      - red_box                         # names of the objects as defined in training or used dataset
```

* The [YOLO network config](../../code/yolo_samples/configuration/darknet_ros/yolo_network_config/cfg) has to be adjusted like mentioned above [(example)](training.md#yolo-configuration)

* Weights have to be placed into `[catkin workspace]/src/darknet_ros/darknet_ros/yolo_network_config/weights`

* The launch file in `[catkin workspace]/src/darknet_ros/darknet_ros/darknet_ros.launch` has to be adjusted for the correct YOLO version:

```
...
  <!-- ROS and network parameter files -->
  <arg name="ros_param_file"             default="$(find darknet_ros)/config/ros.yaml"/>
  <arg name="network_param_file"         default="$(find darknet_ros)/config/yolov3.yaml"/>     # <- version
...
```

## darknet_ros_3d

* edit the config file in `[catkin workspace]/src/gb_visual_detection_3d/darknet_ros_3d/config/darknet_3d.yaml`:

```
...
interested_classes: ["red_box"]                         # custom classes
```

* edit the launch file in `[catkin workspace]/src/gb_visual_detection_3d/darknet_ros_3d/launch`:

```
...
  <!-- ROS and network parameter files -->
  <arg name="ros_param_file"             default="$(find darknet_ros)/config/ros.yaml"/>
  <arg name="network_param_file"         default="$(find darknet_ros)/config/yolov3y.yaml"/>     # <- version
...
```
