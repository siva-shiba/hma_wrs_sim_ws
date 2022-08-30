ROS_MASTER_URI=http://localhost:11311
DOCKER_URI=$(docker inspect -f '{{range.NetworkSettings.Networks}}{{.IPAddress}}{{end}}' hsrb_robocup_dspl_docker_simulator_1)
if [ ${#DOCKER_URI} -gt 0 ]
then
      ROS_MASTER_URI="http://"$DOCKER_URI":11311"
fi

ROS_IP=`hostname -I | cut -d' ' -f1`

WS_CONTAINER_NAME="hma_wrs_sim_ws-hma_wrs_sim_ws-1"

docker exec -it \ 
      --env ROS_MASTER_URI=$ROS_MASTER_URI \
      --env ROS_IP=$ROS_IP \
      $WS_CONTAINER_NAME \
      /entrypoints/roscore.sh