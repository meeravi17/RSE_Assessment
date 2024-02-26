#!/bin/bash

WORKSPACE="~/RSE_Assessment/RSE_ws/devel/setup.bash"

session="offline_localization"
tmux new-session -d -s $session

tmux new-window -t $session -n 'robot_controller' 
tmux send-keys -t 'robot_controller' 'cd ~/RSE_Assessment/RSE_ws' C-m
tmux send-keys -t 'robot_controller' 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/RSE_Assessment/RSE_ws/src/mybot_pkg/models' C-m
tmux send-keys -t 'robot_controller' 'source ~/RSE_Assessment/RSE_ws/devel/setup.bash' C-m
tmux send-keys -t 'robot_controller' 'roslaunch mybot_pkg myrobot.launch' C-m

sleep 5

tmux new-window -t $session -n 'laserscanconvert'
tmux send-keys -t 'laserscanconvert' 'source ~/RSE_Assessment/RSE_ws/devel/setup.bash' C-m
tmux send-keys -t 'laserscanconvert' 'roslaunch pointcloud_to_laserscan pointcloud_to_laserscan.launch ' C-m

tmux new-window -t $session -n 'map_publisher'
tmux send-keys -t 'map_publisher' 'source ~/RSE_Assessment/RSE_ws/devel/setup.bash' C-m
tmux send-keys -t 'map_publisher' 'roslaunch steer_bot_navigation map_publisher.launch ' C-m

tmux new-window -t $session -n 'tf_map'
tmux send-keys -t 'tf_map' 'source ~/RSE_Assessment/RSE_ws/devel/setup.bash' C-m
tmux send-keys -t 'tf_map' 'roslaunch steer_bot_navigation tf_map.launch ' C-m

tmux new-window -t $session -n 'offline_node'
tmux send-keys -t 'offline_node' 'source ~/RSE_Assessment/RSE_ws/devel/setup.bash' C-m
tmux send-keys -t 'offline_node' 'roslaunch steer_bot_navigation offlineICP.launch ' C-m