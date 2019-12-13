#! /bin/bash
tmux splitw -h "rosrun pf_localisation node.py"
tmux splitw -v "rosrun rviz rviz"
