#!/bin/bash

WORKSPACE="~/RSE_Assessment/RSE_ws/devel/setup.bash"

session="test_robot_map_nav"
tmux new-session -d -s $session

tmux new-window -t $session -n 'roscore'
tmux send-keys -t 'roscore' 'roslaunch --core --screen' C-m
tmux set-option -t 'roscore' remain-on-exit

sleep 2

tmux new-window -t $session -n 'flask-gui'
tmux send-keys -t 'flask-gui' 'source ~/RSE_Assessment/RSE_ws/devel/setup.bash' C-m
tmux send-keys -t 'flask-gui' 'python3 ~/RSE_Assessment/RSE_ws/src/robot_web_gui/flask_app.py ' C-m

tmux new-window -t $session -n 'rosbridge_socket'
tmux send-keys -t 'rosbridge_socket' 'roslaunch rosbridge_server rosbridge_websocket.launch ' C-m