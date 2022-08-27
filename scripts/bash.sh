ROS_MASTER_URI=http://localhost:11311
DOCKER_URI=$(docker inspect -f '{{range.NetworkSettings.Networks}}{{.IPAddress}}{{end}}' hsrb_robocup_dspl_docker_simulator_1)
if [ ${#DOCKER_URI} -gt 0 ]
then
      ROS_MASTER_URI="http://"$DOCKER_URI":11311"
fi

WS_CONTAINER_NAME="hma_wrs_sim_ws-hma_wrs_sim_ws-1"

docker exec -it --env ROS_MASTER_URI=$ROS_MASTER_URI --env TERM=xterm-color --workdir /home/ros_ws/hma/hma_wrs_sim_ws $WS_CONTAINER_NAME /entrypoints/bash.sh
