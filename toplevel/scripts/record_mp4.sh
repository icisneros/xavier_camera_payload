#! /bin/bash

# echo "debug check ..."
# timeout 10s rostopic hz /fix
# read -p "Is the GPS working properly?:" flag
# if [ $flag in [Yy] ]; then
#     echo "Do camera check"
#     export DISPLAY="127.0.0.1:10.0"
#     timeout 15s rqt_image_view
#     export GPS_SUCCESS=1
# else
#     echo "Restart the payload!"
#     export GPS_SUCCESS=0
# fi
# if [$GPS_SUCCESS -eq 1]; then
roslaunch nvidia_to_ros field_record_camera.launch