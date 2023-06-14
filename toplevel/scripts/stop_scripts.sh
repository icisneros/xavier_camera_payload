#!/bin/bash

# Kill the localization window (from last window to first)
SESSION="Localization_session"
WINDOW="Localization_window"

for pane_id in 7 6 5 4 2 1 3 0; do
    tmux send-keys -t ${SESSION}:${WINDOW}.${pane_id} C-b C-c 
    sleep 2
done
sleep 5
tmux kill-session -t $SESSION



# Kill the sensors (from last window to first)
SESSION="sensors_session"
WINDOW="sensors_window"

for pane_id in 6 5 4 3 2 1 0; do
    tmux send-keys -t ${SESSION}:${WINDOW}.${pane_id} C-b C-c 
    sleep 2
done
sleep 5
tmux kill-session -t $SESSION