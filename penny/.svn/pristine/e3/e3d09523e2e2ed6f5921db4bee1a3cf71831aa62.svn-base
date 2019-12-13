#! /bin/bash
tmux splitw -h "roscore"
sleep 1s
tmux splitw -v "rosrun stage_ros stageros newlowerground.world"
tmux splitw -v "rosrun map_server map_server newlowerground.yaml"
tmux splitw -v "roslaunch socspioneer keyboard_teleop.launch"
