# YOLOv3 Installation and custom object training
This section includes instructions for detecting objects in custom images for use with the YOLOv3 neural network. It uses Alexey Bochkovskiys fork of YOLO and Darknet.

## 1 Cloning Darknet

The next sections will clone the Darknet framework from Alexey Bochkovskiy fork of YOLO which besides YOLO v4 includes several [improvements](https://github.com/AlexeyAB/darknet#improvements-in-this-repository).

### Clone the repository to this machine

`<destination path>` is the path of the cloned repository.

```
$ git clone https://github.com/AlexeyAB/darknet <destination path>
```
### Enable OPENCV and GPU acceleration (only if supported by the GPU)

#### Change the configuration

```
$ sed -i 's/OPENCV=0/OPENCV=1/' <destination path>/Makefile
$ sed -i 's/GPU=0/GPU=1/' <destination path>/Makefile
$ sed -i 's/CUDNN=0/CUDNN=1/' <destination path>/Makefile
```

#### Install CUDA

To use CUDA, it has to be installed using the official [instructions](https://developer.nvidia.com/cuda-downloads?target_os=Linux&target_arch=x86_64&Distribution=Ubuntu&target_version=20.04&target_type=deb_local):

```
$ wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-ubuntu2004.pin
# mv cuda-ubuntu2004.pin /etc/apt/preferences.d/cuda-repository-pin-600
$ wget https://developer.download.nvidia.com/compute/cuda/11.6.0/local_installers/cuda-repo-ubuntu2004-11-6-local_11.6.0-510.39.01-1_amd64.deb
# dpkg -i cuda-repo-ubuntu2004-11-6-local_11.6.0-510.39.01-1_amd64.deb
# apt-key add /var/cuda-repo-ubuntu2004-11-6-local/7fa2af80.pub
# apt-get update
$ export PATH=/usr/local/cuda-11.5/bin${PATH:+:${PATH}} # replace version number
$ echo export PATH=/usr/local/cuda-11.5/bin${PATH:+:${PATH}} >> $HOME/.bashrc # adjust if another shell or CUDA version is used
```

To install the required packages, use

```
# apt install nvidia-driver-<recommended version> # you can also use Ubuntu's "Additional Drivers" program
# apt install cuda cuda-nvcc-<version>
```


### Build Darknet

```
$ cd <destination path>
$ make
```

## 2 Download pretrained weights

Download pretrained weights for general object detection and tests.

### Download weights from pjreddie.com (official source for original YOLO until v3)

```
$ cd <destination path>
$ wget https://pjreddie.com/media/files/yolov3.weights
```

## 3 Detect objects using YOLOv3

YOLOv3 can detect the following classes without further configuration: [COCO classes](http://cocodataset.org/#explore)

### Detect and show sample image

```
$ <destination path>/darknet detect <destination path>/cfg/yolov3.cfg <destination path>/yolov3.weights <destination path>/data/person.jpg
```


# Adding custom objects to the detector

This section can be used to add custom objects to be detected.

## 1 Labeling

Labeled images are necessary for training an object in YOLO. Images need bounding boxes and object names to be useful.

To label images, an annotation tool such as [LabelImg](https://github.com/tzutalin/labelImg) or any other tool supporting YOLO can be used. Bounding boxes should be as close to the objects as possible.

Move the images and label files to `<destination path>/data/obj`. The current version available on the `pythin-pip` package manager has a bug that appends an unnecessary `.xml` extension between the filename and the actual file extension. Darknet needs files with a `.txt` extension to work correctly (`sample.xml.txt` should be `sample.txt`). To rename all files, you can use this bash script:

```
#!/usr/bin/env bash

for f in *.xml.txt; do 
    mv -- "$f" "${f%.xml.txt}.txt"
done
```

## YOLO configuration

Edit the `<destination path>/yolo.cfg` file as follows (save it as yolov3_custom.cfg):

### [net]

* `batch = 64`
* `subdivisions = 16` (`32` if issues occur)
* `max_batches = 10000` (higher number leads to greater accuracy but longer compute time)
* `max_batches = (2000 * number_of_classes, minimum 4000)`
* `steps = (0.8 * max_batches), (0.9 * max_batches)`

### [convolutional] (all, above [yolo]!)

* `filters = (number_of_classes + 5) * 3`

### [yolo] (all)

* `classes = number_of_classes`

### Optional

In each yolo layer in the configuration file, change `random = 1` to `random = 0` to speed up training but slightly reduce accuracy of model. Will also help save memory if you run into any memory issues.

## Object configuration

Create a file under `<destination path>/obj.names` and put in the names of all custom objects, exactly as in the `classes.txt` file.

Create another file in `<destination path>/obj.data` with the following contents:

```
classes = number_of_classes (replace with actual number)
train = data/train.txt
valid = data/test.txt
names = data/obj.names
```

## Train.txt file

The `train.txt` file specifies the paths of the training images

The following Python script can be used to create that file automatically: 

```
#!/usr/bin/env python

import os

image_files = []
os.chdir(os.path.join("data", "obj"))
for filename in os.listdir(os.getcwd()):
    if filename.endswith(".jpg"):
        image_files.append("data/obj/" + filename)
os.chdir("..")
with open("train.txt", "w") as outfile:
    for image in image_files:
        outfile.write(image)
        outfile.write("\n")
    outfile.close()
os.chdir("..")
```

Source: https://github.com/theAIGuysCode/YoloGenerateTrainingFile/blob/master/generate_train.py/


## 3 Download pre-trained convolutional layer weights

Pretrained weights for the convolutional layers can improve training speed and accuracy of the algorithm. Those weights work best with objects included in the COCO dataset. They are optional.

```
$ wget http://pjreddie.com/media/files/darknet53.conv.74 <destination path>
```

## 4 Executing the custom object training

Use this command to start the custom object training:

```
$ <destination path>/darknet detector train <destination path>/data/obj.data <destination path>/cfg/yolov3_custom.cfg <destination path>/darknet53.conv.74
```

A chart of average loss vs iterations is saved as `<destination path>/chart.png`.
