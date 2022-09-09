FROM ghcr.io/haganelego/hma_ros_docker/hma_ros:melodic-cuda11.2.2-cudnn8-simulator

SHELL [ "/bin/bash", "-c" ]

ENV HOME /home

# create workspace folder
RUN mkdir -p $HOME/ros_ws/hma/hma_wrs_sim_ws
RUN mkdir -p $HOME/ros_ws/hma/cv_bridge_ws

# install dependencies defined in package.xml
RUN rosdep init && rosdep update

# build cv_bridge, image_geometry
RUN source /opt/ros/melodic/setup.bash && \
    cd $HOME/ros_ws/hma/cv_bridge_ws && \
    mkdir install && \
    mkdir src && \
    catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so && \
    catkin config --install && \
    git clone https://github.com/ros-perception/vision_opencv.git src/vision_opencv && \
    cd src/vision_opencv/ && \
    git checkout 1.13.0 && \
    cd ../../ && \
    catkin build cv_bridge && \
    catkin build image_geometry 

# # copy files
ADD ./scripts $HOME/ros_ws/hma/hma_wrs_sim_ws/scripts
ADD ./src/03_tmc/deb $HOME/setup/deb

# install hsrb package
RUN cd $HOME/setup/deb && \
    dpkg -i ros-melodic-tarp3_0.6.0-1.bionic.20210617.2331.+0000_amd64.deb && \
    dpkg -i ros-melodic-tarp3-urdf_0.6.0-1.bionic.20210617.2331.+0000_amd64.deb && \
    dpkg -i ros-melodic-tmc-navigation-msgs_0.24.0-1.bionic.20211108.0234.+0000_amd64.deb && \
    dpkg -i ros-melodic-tmc-pose-2d-lib_0.30.1-1.bionic.20211108.0503.+0000_amd64.deb && \
    dpkg -i ros-melodic-tmc-omni-path-follower_0.30.1-1.bionic.20211108.0503.+0000_amd64.deb && \
    dpkg -i ros-melodic-tmc-control-msgs_0.9.2-1.bionic.20211108.0259.+0000_amd64.deb && \
    dpkg -i ros-melodic-tmc-manipulation-types_0.32.1-1.bionic.20211108.0324.+0000_amd64.deb && \
    dpkg -i ros-melodic-tmc-robot-kinematics-model_0.32.1-1.bionic.20211108.0324.+0000_amd64.deb && \
    dpkg -i ros-melodic-hsrb-analytic-ik_0.29.4-1.bionic.20211108.0549.+0000_amd64.deb && \
    dpkg -i ros-melodic-hsrb-parts-description_0.20.5-1.bionic.20211104.0057.+0000_amd64.deb && \
    dpkg -i ros-melodic-hsrb-description_0.20.5-1.bionic.20211104.0057.+0000_amd64.deb && \
    dpkg -i ros-melodic-hsrb-moveit-config_1.0.0-2.bionic.20220309.2325.+0000.29cd601_amd64.deb && \
    dpkg -i ros-melodic-hsrb-moveit-plugins_0.6.3-2.bionic.20220309.2325.+0000.29cd601_amd64.deb && \
    dpkg -i ros-melodic-hsrb-moveit_0.6.3-2.bionic.20220309.2325.+0000.29cd601_amd64.deb

# bashrc settings
RUN echo "source /opt/ros/melodic/setup.bash" >> /root/.bashrc && \
    echo "source /root/.bashrc_hma" >> /root/.bashrc

RUN echo "source /opt/ros/melodic/setup.bash" >> /root/.bashrc && \
    echo "source /home/ros_ws/hma/cv_bridge_ws/install/setup.bash --extend" >> /root/.bashrc && \
    echo "source /home/ros_ws/hma/hma_wrs_sim_ws/devel/setup.bash" >> /root/.bashrc

# Please edit from here
RUN rm /etc/apt/sources.list.d/cuda.list
RUN apt-key del 7fa2af80 && \
    curl -O https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/cuda-keyring_1.0-1_all.deb && \
    dpkg -i cuda-keyring_1.0-1_all.deb
RUN apt-get -y update
RUN apt-get -y install ros-melodic-rqt-*
RUN apt-get -y install ros-melodic-hector-slam
RUN apt-get -y install ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control
# Do not edit from here

# COPY ./docker-entrypoint.sh /docker-entrypoint.sh
# RUN chmod +x /docker-entrypoint.sh
# ENTRYPOINT ["/docker-entrypoint.sh"]
ENTRYPOINT ["bash"]
