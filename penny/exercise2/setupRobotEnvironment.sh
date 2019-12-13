#! /bin/bash
tmux splitw -h "roscore"
sleep 1s
tmux splitw -v "rosrun map_server map_server newlowerground.yaml"
tmux splitw -v "roslaunch socspioneer p2os_teleop_joy.launch"
