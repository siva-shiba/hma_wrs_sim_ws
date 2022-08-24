#!/bin/bash

cd /home/ros_ws/hma/hma_wrs_sim_ws
source /opt/ros/melodic/setup.bash
cd /home/ros_ws/hma/hma_wrs_sim_ws
catkin build --no-status

TERM=xterm-color
source /opt/ros/melodic/setup.bash
source /home/ros_ws/hma/cv_bridge_ws/install/setup.bash --extend
source /home/ros_ws/hma/hma_wrs_sim_ws/devel/setup.bash
echo "source /root/.bashrc_hma" >> /root/.bashrc
bash --rcfile ~root/.bashrc