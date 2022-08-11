#!/bin/bash

TERM=xterm-color
source /opt/ros/melodic/setup.bash
source /home/ros_ws/hma/cv_bridge_ws/install/setup.bash --extend
source /home/ros_ws/hma/hma_wrs_sim_ws/devel/setup.bash
bash --rcfile ~root/.bashrc
# PS1='${debian_chroot:+($debian_chroot)}\[\033[01;33m\]\u@\h\[\033[00m\]:\[\033[01;35m\]\w\[\033[00m\]\$ '
# PS1='${debian_chroot:+($debian_chroot)}\[\033[30m\]\[\033[43m\][ros_mode]\[\033[01;49m\] \[\033[01;33m\]\u@\h\[\033[00m\]:\[\033[01;35m\]\w\[\033[00m\]\$ '
