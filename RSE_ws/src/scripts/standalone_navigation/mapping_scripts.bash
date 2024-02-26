#!/bin/bash

WORKSPACE="~/RSE_Assessment/RSE_ws/devel/setup.bash"

session="mapping_bash"
tmux new-session -d -s $session

tmux new-window -t $session -n 'robot_controller' 
tmux send-keys -t 'robot_controller' 'cd ~/RSE_Assessment/RSE_ws' C-m
tmux send-keys -t 'robot_controller' 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/RSE_Assessment/RSE_ws/src/mybot_pkg/models' C-m
tmux send-keys -t 'robot_controller' 'source ~/RSE_Assessment/RSE_ws/devel/setup.bash' C-m
tmux send-keys -t 'robot_controller' 'roslaunch mybot_pkg myrobot_mapping.launch' C-m

sleep 2

tmux new-window -t $session -n 'mapping'
tmux send-keys -t 'mapping' 'source /opt/ros/noetic/setup.bash' C-m
tmux send-keys -t 'mapping' 'roslaunch rtabmap_demos demo_husky.launch' C-m
