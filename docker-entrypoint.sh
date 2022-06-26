#!/bin/bash
echo "========== HMA WRS Sim Start =========="
source /opt/ros/melodic/setup.bash && cd /home/ros_ws/hma/hma_wrs_sim_ws && catkin build --no-status
source /home/ros_ws/hma/cv_bridge_ws/install/setup.bash --extend && source /home/ros_ws/hma/hma_wrs_sim_ws/devel/setup.bash
echo "Start smach"
echo $ROS_MASTER_URI
sh /home/ros_ws/hma/hma_wrs_sim_ws/scripts/exec_all.sh
tail -f /dev/null