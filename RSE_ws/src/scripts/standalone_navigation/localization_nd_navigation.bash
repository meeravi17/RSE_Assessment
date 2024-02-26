#!/bin/bash

echo "Map_file: $@"
session="navigation_bash"
tmux new-session -d -s $session

tmux new-window -t $session -n 'roscore'
tmux send-keys -t 'roscore' 'roslaunch --core --screen' C-m
tmux set-option -t 'roscore' remain-on-exit

sleep 1
tmux new-window -t $session -n 'robot_controller' 
tmux send-keys -t 'robot_controller' 'cd ~/RSE_Assessment/RSE_ws' C-m
tmux send-keys -t 'robot_controller' 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/RSE_Assessment/RSE_ws/src/mybot_pkg/models' C-m
tmux send-keys -t 'robot_controller' 'source ~/RSE_Assessment/RSE_ws/devel/setup.bash' C-m
tmux send-keys -t 'robot_controller' 'roslaunch mybot_pkg myrobot.launch' C-m

sleep 2
tmux new-window -t $session -n 'laserscanconvert'
tmux send-keys -t 'laserscanconvert' 'source ~/RSE_Assessment/RSE_ws/devel/setup.bash' C-m
tmux send-keys -t 'laserscanconvert' 'roslaunch pointcloud_to_laserscan pointcloud_to_laserscan.launch ' C-m

sleep 1
tmux new-window -t $session -n 'navigation'
tmux send-keys -t 'navigation' 'source ' 'source ~/RSE_Assessment/RSE_ws/devel/setup.bash' C-m
tmux send-keys -t 'navigation' 'roslaunch steer_bot_navigation loc_and_mapping.launch map_file:="$(pwd)"/static/'"$1"'.yaml' C-m

tmux new-window -t $session -n 'remapper'
tmux send-keys -t 'remapper' 'source ~/RSE_Assessment/RSE_ws/devel/setup.bash' C-m
tmux send-keys -t 'remapper' 'roslaunch steer_bot_navigation cmd_vel_remapper.launch' C-m