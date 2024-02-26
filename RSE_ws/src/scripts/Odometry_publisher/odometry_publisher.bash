#!/bin/bash

WORKSPACE="~/RSE_Assessment/RSE_ws/devel/setup.bash"

session="odometry_publisher"
tmux new-session -d -s $session

tmux new-window -t $session -n 'roscore'
tmux send-keys -t 'roscore' 'roslaunch --core --screen' C-m
tmux set-option -t 'roscore' remain-on-exit

sleep 2

tmux new-window -t $session -n 'odometry_publisher'
tmux send-keys -t 'odometry_publisher' 'source ~/RSE_Assessment/RSE_ws/devel/setup.bash' C-m
tmux send-keys -t 'odometry_publisher' 'roslaunch steer_bot_navigation odometry_publisher.launch' C-m
