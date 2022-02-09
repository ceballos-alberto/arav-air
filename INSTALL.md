# Installation Instructions and required libraries

In order to install this repository, follow the steps outlined below. The installation has been tested and is only functional in Ubuntu 20.04
	
## Install ROS Noetic

[ROS Noetic Installation](http://wiki.ros.org/noetic/Installation 'Install ROS Noetic')

## Install Octomap Libraries

`sudo apt-get install ros-noetic-octomap ros-noetic-octomap-mapping ros-noetic-octomap-msgs ros-noetic-octomap-ros ros-noetic-octomap-rviz-plugins ros-noetic-octomap-server`

## Install ros control

`sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers`

## Install and update pip

`sudo apt install python3-pip`

`pip install --upgrade pip`

## Install OpenCV

`sudo apt install libgl1-mesa-glx ffmpeg libsm6 libxext6`

`pip install opencv-contrib-python`

## Install TensorFlow

[TensorFlow installation](https://www.tensorflow.org/install 'Install TensorFlow')

To install you don't need the website, only need to do:

`pip install tensorflow`

## Install ompl library

`sudo apt install libompl-dev`

## Install FCL library

First install boost library

`sudo apt-get install libboost-all-dev`

Then install libccd

`sudo apt-get install libccd-dev`

Finally install the library

Follow the instructions on its README and USE VERSION 0.5

[Flexible Collision Library](https://github.com/flexible-collision-library/fcl/releases 'Install FCL')

## Install Mavros library and dependencies

`sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras ros-noetic-mavros-msgs`

`wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh`

`sudo bash ./install_geographiclib_datasets.sh`

## Install PX4 dependencies

`pip3 install kconfiglib`

`pip3 install --user jinja2`

`pip3 install --user packaging`

`pip3 install --user toml`

`pip3 install --user jsonschema`

`sudo apt install ant`

`sudo apt install openjdk-11-jdk`

`sudo apt-get install libgstreamer-plugins-base1.0-dev`

`sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y`

