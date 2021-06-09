#!/usr/bin/env bash
echo " ============================================================================="
echo "| Stopping the deepracer-core.service that is currently running on the device |"
echo " ============================================================================="
systemctl stop deepracer-core
echo ""
echo " ============================================================"
echo "| Downloading the dependencies of Follow the Leader packages |"
echo " ============================================================"
source /opt/ros/foxy/setup.bash 
source /opt/intel/openvino_2021/bin/setupvars.sh
cd ~/deepracer_ws/aws-deepracer-follow-the-leader-sample-project/deepracer_follow_the_leader_ws/ 
./install_dependencies.sh
rosws update
rosdep install -i --from-path . --rosdistro foxy -y
echo ""
echo " ========================================="
echo "| Building the Follow the Leader packages |"
echo " ========================================="
colcon build
echo ""
echo " ================================================================"
echo "| Downloading and building Follow the Leader packages completed  |"
echo " ================================================================"
