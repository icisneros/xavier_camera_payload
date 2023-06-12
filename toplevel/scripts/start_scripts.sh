#!/bin/bash
# Launch two terminals
# Sensors launch and rosbag record
gnome-terminal --window --maximize --title="Start Sensors" -- bash -c "tmuxp load /home/airlab/ws/src/toplevel/sensors_start.yaml"

# Launch localization scripts (docker and odom)
gnome-terminal --window --maximize --title="Start Localization" -- bash -c "sleep 20; tmuxp load /home/airlab/GNSS_Denied_Localization_IMU/src/UAV_VPR_loc/launch/Localization_start.yaml"