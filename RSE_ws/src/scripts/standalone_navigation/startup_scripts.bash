#!/bin/bash

WORKSPACE="~/RSE_Assessment/RSE_ws/devel/setup.bash"

session="test_robot_map_nav"
tmux new-session -d -s $session

tmux new-window -t $session -n 'roscore'
tmux send-keys -t 'roscore' 'roslaunch --core --screen' C-m
tmux set-option -t 'roscore' remain-on-exit
sleep 2

tmux new-window -t $session -n 'gazebo'
tmux send-keys -t 'gazebo' 'source ~/RSE_Assessment/RSE_ws/devel/setup.bash' C-m
tmux send-keys -t 'gazebo' 'roslaunch mybot_pkg world.launch' C-m
