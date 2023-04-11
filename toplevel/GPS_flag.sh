#!/bin/bash
rm GPS_SUCCESS.txt
timeout 10s rostopic hz /fix
while true; do
    read -p "Is GPS working properly? (y/n) " yn
    case $yn in
        [Yy]* ) echo GPS working properly;
                echo 1 >> GPS_SUCCESS.txt;
                break;;
        [Nn]* ) echo GPS working inproperly, restart!;exit;;
        * ) echo "Invalid input.";;
    esac
done
GPS_SUCCESS=$(<GPS_SUCCESS.txt)
echo $GPS_SUCCESS
if [ $GPS_SUCCESS -eq 1 ]
then
    echo "Do camera check"
    timeout 10s rqt_image_view
else
    echo "Restart the payload!"
fi
