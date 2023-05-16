#!/bin/bash
SESSION="mysession"
WINDOW="mywindow"

for pane_id in 7 6 5 4 3 2 1 0; do
    tmux send-keys -t ${SESSION}:${WINDOW}.${pane_id} C-b C-c 
    sleep 2
done

tmux kill-session -t $SESSION

# SESSION="record"
# WINDOW="window"

# for pane_id in 4 2 1 3 0; do
#     tmux send-keys -t ${SESSION}:${WINDOW}.${pane_id} C-b C-c 
#     sleep 5
# done

# tmux kill-session -t $SESSION
