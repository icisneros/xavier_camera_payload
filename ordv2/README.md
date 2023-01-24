# Instructions #
Once the xavier is setup (see below) you can get and build the code as follows:

## Get the workspace setup ##

```
mkdir -p ~/ros/ordv2/src/
cd ~/ros/ordv2/src/
git clone git@bitbucket.org:castacks/ordv2.git
ln -s ordv2/workspace/rosinstall .rosinstall
wstool up
wstool info
```

## Build ##
```
cd ~/ros/ordv2/
catkin_make
```

## Run all Drivers (IMU, Pixracer, Buttons, Lights, Cameras) ##
```
cd ~/ros/ordv2/
source devel/setup.bash
roslaunch ordv2 hardware.launch
```


# Setup a new xavier nx with all the required packages and changes#

## Add ROS repo:##

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
apt-get update
```

## Install dependencies:##
```
apt install cmake libgtk-3-dev libjpeg-dev libgles2-mesa-dev libgstreamer1.0-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-base libgstreamer-plugins-bad1.0-dev gstreamer1.0-plugins-bad  python-catkin-pkg python-empy python-nose python-setuptools libgtest-dev python-rosinstall python-rosinstall-generator python-wstool build-essential git minicom chrony v4l-utils  libgoogle-glog-dev libgflags-dev libatlas-base-dev libeigen3-dev libsuitesparse-dev
```

## Install CERES from source: ##
```
http://ceres-solver.org/installation.html
```

## Install ROS:##
```
sudo apt install ros-melodic-desktop 
```

## Install some more packages: ##
```
sudo apt install ros-melodic-mavros ros-melodic-pcl-ros ros-melodic-vision-opencv ros-melodic-camera-info-manager ros-melodic-camera-info-manager-py python3-vcstool
```

## Setup Rosdep: ##
```
sudo apt install python-rosdep
rosdep update
rosdep install
```


## Misc Setup: ##

### Build Argus Samples and Test Camera###

```
cd /usr/src/jetson_multimedia_api/argus
cmake .
make
cd apps/camera/ui/camera
```
Test that the camera works:
```
./argus_camera
```

### Add user to dialout group to use serial ports ###

```
sudo adduser airlab dialout
```
### Install Geographic Datasets for MAVROS ###

```
sudo /opt/ros/melodic/lib/mavros/install_geographiclib_datasets.sh

```


### Fix OpenCV: ###
```
sudo ln -s /usr/include/opencv4/opencv2/ /usr/include/opencv
```

### Enable the power button to shutdown the computer: ###
```
gsettings set org.gnome.settings-daemon.plugins.power button-power 'shutdown'
```


### Setup the timesync with the r5 code for PPS ###

```
cd ~/ros/ordv2/src/mmpug_r5timesync/syncr5clock
make
sudo make install
```
Reboot after the install. The program syncr5clock should now be running as a service.
