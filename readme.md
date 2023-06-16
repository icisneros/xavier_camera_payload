# SETTING UP A NEW XavierNX 

<!-- # Current hardware
airlab@linux:~/jetsonUtilities$ python3 jetsonInfo.py 
NVIDIA NVIDIA Jetson Xavier NX Developer Kit
 L4T 32.7.1 [ JetPack 4.6.1 ]
   Ubuntu 18.04.6 LTS
   Kernel Version: 4.9.253-tegra
 CUDA 10.2.300
   CUDA Architecture: 7.2
 OpenCV version: 4.1.1
   OpenCV Cuda: NO
 CUDNN: 8.2.1.32
 TensorRT: 8.2.1.8
 Vision Works: 1.6.0.501
 VPI: 1.2.3
 Vulcan: 1.2.70 -->



# (0) Disable lock screen
# All Settings > Brightness and Lock > Lock (OFF)
# Uncheck "Require my password when waking from suspend"
# Turn screen off when inactive for > Never
gsettings set org.gnome.desktop.lockdown disable-lock-screen 'true'
# All Settings > Security and Privacy > Password Settings > Automatic Login (ON)

# (0.1) Misc other libraries that are needed
sudo apt install nano
sudo apt-get install tmux
pip3 install tmuxp
# jtop
sudo pip3 install -U jetson-stats

# (1) Create and setup new workspace for the sensor packages.
sudo apt-get install python3-pip
pip3 install catkin-pkg==0.5.2
mkdir -p ~/ws/src
cd ~/ws
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
# Clone the repo
cd ~
git clone https://github.com/icisneros/xavier_camera_payload.git
# Copy over the packages from the xavier_camera_payload folder into your newly created workspace
# Copy them to ~/ws/src/
# Next:
sudo apt install python-pip
pip install empy
cd ~/ws/src/
catkin_make

# Update the permissions of the UART ports for the GPS and IMU
sudo chmod 666 /dev/ttyTHS0
sudo chmod 666 /dev/ttyTHS2

# Test the sensors (this launches the camera and the IMU)
cd ~/ws
source devel/setup.bash
roscore
rqt_image_view
roslaunch ~/ws/src/toplevel/launch/master_calib_single_camera.launch

# Once everything is working, add this workspace to the bashrc
nano ~/.bashrc
# Add the following lines underneath the "source /opt/..." line:
export PATH=/usr/local/cuda-10.2/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-10.2/lib64:$LD_LIBRARY_PATH
source /home/airlab/ws/devel/setup.bash



# (2)  Clone the repo with the python ROS nodes and the docker information
cd ~/GNSS_Denied_Localization_IMU/src
git clone https://github.com/icisneros/UAV_VPR_loc.git
cd UAV_VPR_loc/
git checkout darpa_dino_xavier1_async


# (3) Move default docker files to the nvme SSD. Then download the docker image.
# Add user to the docker group
sudo usermod -aG docker $USER
# Shutdown or logout and login for changes to take effect
# Move the files and create a symlink
service docker stop
sudo tar -zcC /var/lib docker > /home/airlab/data/var_lib_docker-backup-2023-06-16.tar.gz
sudo mv /var/lib/docker /home/airlab/data/docker
sudo ln -s /home/airlab/data/docker /var/lib/docker
sudo ls /var/lib/docker/
service docker start

# Download and setup the docker image
docker pull theairlab/gnss-denied:noetic-pytorch-l4t-r32.7.1

# Test whether the docker image runs
cd /home/airlab/GNSS_Denied_Localization_IMU/src/UAV_VPR_loc/docker
bash run_docker.sh


# (4) Setup the button script to run on startup
cd /home/airlab/ws/src/toplevel
chmod +x button.py
mkdir -p ~/.config/autostart
cp /home/airlab/ws/src/toplevel/launch.desktop /home/airlab/.config/autostart/