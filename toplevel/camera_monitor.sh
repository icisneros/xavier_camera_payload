#!/bin/bash

declare -A pane_output # associative array to store previous output of each pane


while true; do
  has_stopped=0
  pane_id=1  # pane for camera launch 
  cam_rec_pane_id=3 # pane for camera logging 
  output=$(tmux capture-pane -p -t mysession:mywindow.$pane_id)
  previous_output=${pane_output[$pane_id]}
  if [ "$output" != "$previous_output" ]; then
    pane_output[$pane_id]=$output
    echo "$output"
  else
    has_stopped=1
  fi

  if [ $has_stopped -eq 1 ]; then
    # If the output in any pane has stopped changing, run your command here
    echo "Output in a pane has stopped changing"
    # Ctrl c  the camera launch file
    tmux send-keys -t mysession:mywindow.$pane_id C-b C-c
    tmux send-keys -t mysession:mywindow.$pane_id C-b C-c
    # sleep 20   # very conservative wait time for command to stop


    # Ctrl c  the camera record file
    tmux send-keys -t mysession:mywindow.$cam_rec_pane_id C-b C-c
    tmux send-keys -t mysession:mywindow.$cam_rec_pane_id C-b C-c
    sleep 20   # very conservative wait time for command to stop


    # camera system restart
    tmux send-keys -t mysession:mywindow.$pane_id "sudo rm -rf /tmp/image_servers/ /dev/shm/sem.image.sem" Enter
    sleep 3
    
    # Search for a password prompt in the output
    output=$(tmux capture-pane -p -t mysession:mywindow.$pane_id)
    if [[ $output == *"[sudo] password for airlab:"* ]]; then
      tmux send-keys -t mysession:mywindow.$pane_id "passme24" Enter
    else
      echo "Pane is not requesting a password"
    fi

    tmux send-keys -t mysession:mywindow.$pane_id "sudo systemctl restart nvargus-daemon.service" Enter

    break


    sleep 5


    # Run launch files again
    # Run camera
    tmux send-keys -t mysession:mywindow.$pane_id "roslaunch nvidia_to_ros argus_camera.launch" Enter


    sleep 15


    # Run the camera recording
    tmux send-keys -t mysession:mywindow.$cam_rec_pane_id "roslaunch nvidia_to_ros field_record_camera.launch" Enter




  fi

  sleep 1 # Adjust the sleep time as needed
done