## AI-based robot control system with object detection and localization using Industrial ROS

This repo contains my bachelor thesis (German) I wrote during my studies in mechatronics.
It uses [ROS Industrial](https://rosindustrial.org/) for recognizing different objects and their position using an
[Intel RealSense D435](https://www.intelrealsense.com/depth-camera-d435/).

The real-time neural network for object detection [YOLOv3](https://pjreddie.com/darknet/yolo/)
is used to recognize different objects using the RGB camera of the RealSense and get their
position from the 3D sensor.

### 📹 Demo

[![Demo: Scan, Plan and Build](http://img.youtube.com/vi/Fik9h4zRARM/0.jpg)](http://www.youtube.com/watch?v=Fik9h4zRARM "Demo: Scan, Plan and Build")

### 📜 Abstract

> This thesis describes the development of an AI-based robot controller using
> ROS. It uses a 3D camera system to recognize the position and class of unordered
> objects. To accomplish this, an Intel RealSense 435 3D camera system and
> the object recognition algorithm YOLOv3 are used. The system is implemented
> on the six-axis articulated arm robot RV6L from Reis Robotics. A ROS package
> that matches the two-dimensional object position with the three-dimensional
> image of the RealSense is implemented to determine the distance between
> the camera and the object center. The YOLOv3 algorithm has a low error rate
> in classification and reliably detects the positions of the objects, but cannot
> determine their orientation. Compared to traditional computer vision, the Deep
> Learning algorithm offers advantages especially in terms of the learning process
> and possible further development.

### ⚙️ Developement and Testing

Detailed instructions to set up an environment can be found in the [**project wiki**](https://github.com/justsomescripts/bachelors-thesis/wiki).

### 📁 Files and directories

- [thesis.pdf](docs/thesis.pdf) Final document
- [docs](docs/detailed_instructions/README.md) Documentation / details (clone of [the wiki](https://github.com/justsomescripts/bachelors-thesis/wiki))
- [documentation.html](docs/documentation.html) Documentation / details (html)
- [latex_source/](docs/latex_source/) Thesis source code
- [attachments/](docs/attachments/) Thesis attachments (code and configs)
