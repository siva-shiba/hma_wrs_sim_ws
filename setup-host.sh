ROS_IP=`hostname -I | cut -d' ' -f1`
cd src/04_sim_docker/hsrb_robocup_dspl_docker/ && source set-rosmaster.sh
cd ../../../