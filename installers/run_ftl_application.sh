#!/usr/bin/env bash
echo " ============================"
echo "| Sourcing the setup scripts |"
echo " ============================"
source /opt/ros/foxy/setup.bash 
source /opt/intel/openvino_2021/bin/setupvars.sh
source ~/deepracer_ws/aws-deepracer-follow-the-leader-sample-project/deepracer_follow_the_leader_ws/install/setup.bash
echo ""
echo " ============================================="
echo "| Launching the Follow the Leader application |"
echo "| Use Ctrl+C to kill the nodes                |"
echo " ============================================="
ros2 launch ftl_launcher ftl_launcher.py