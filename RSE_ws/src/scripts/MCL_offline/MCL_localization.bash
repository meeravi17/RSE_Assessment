#!/bin/bash

WORKSPACE="~/RSE_Assessment/RSE_ws/devel/setup.bash"

session="MCL_localization"
tmux new-session -d -s $session

tmux new-window -t $session -n 'robot_bringuo' 
tmux send-keys -t 'robot_bringuo' 'cd ~/RSE_Assessment/RSE_ws' C-m
tmux send-keys -t 'robot_bringuo' 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/RSE_Assessment/RSE_ws/src/mybot_pkg/models' C-m
tmux send-keys -t 'robot_bringuo' 'source ~/RSE_Assessment/RSE_ws/devel/setup.bash' C-m
tmux send-keys -t 'robot_bringuo' 'roslaunch mybot_pkg myrobot.launch' C-m

sleep 5

tmux new-window -t $session -n 'laserscanconvert'
tmux send-keys -t 'laserscanconvert' 'source ~/RSE_Assessment/RSE_ws/devel/setup.bash' C-m
tmux send-keys -t 'laserscanconvert' 'roslaunch pointcloud_to_laserscan pointcloud_to_laserscan.launch ' C-m

tmux new-window -t $session -n 'tf_map'
tmux send-keys -t 'tf_map' 'source ~/RSE_Assessment/RSE_ws/devel/setup.bash' C-m
tmux send-keys -t 'tf_map' 'roslaunch steer_bot_navigation tf_map.launch ' C-m

tmux new-window -t $session -n 'offline_mcl'
tmux send-keys -t 'offline_mcl' 'source ~/RSE_Assessment/RSE_ws/devel/setup.bash' C-m
tmux send-keys -t 'offline_mcl' 'roslaunch steer_bot_navigation MCL_localizer.launch' C-m

tmux new-window -t $session -n 'scan_match'
tmux send-keys -t 'scan_match' 'source ~/RSE_Assessment/RSE_ws/devel/setup.bash' C-m
tmux send-keys -t 'scan_match' 'rosrun steer_bot_navigation match_estimator.py' C-m