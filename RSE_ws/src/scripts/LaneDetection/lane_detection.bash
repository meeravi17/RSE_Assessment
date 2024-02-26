#!/bin/bash

WORKSPACE="~/RSE_Assessment/RSE_ws/devel/setup.bash"

session="lane_navigatio"
tmux new-session -d -s $session

tmux new-window -t $session -n 'robot_controller' 
tmux send-keys -t 'robot_controller' 'cd ~/RSE_Assessment/RSE_ws' C-m
tmux send-keys -t 'robot_controller' 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/RSE_Assessment/RSE_ws/src/mybot_pkg/models' C-m
tmux send-keys -t 'robot_controller' 'source ~/RSE_Assessment/RSE_ws/devel/setup.bash' C-m
tmux send-keys -t 'robot_controller' 'roslaunch mybot_pkg myrobot_navigation.launch' C-m

sleep 5

tmux new-window -t $session -n 'lane detector'
tmux send-keys -t 'lane detector' 'source ~/RSE_Assessment/RSE_ws/devel/setup.bash' C-m
tmux send-keys -t 'lane detector' 'rosrun steer_bot_navigation LaneDetection.py  ' C-m