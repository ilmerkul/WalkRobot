FROM ros:jazzy

ENV DEBIAN_FRONTEND noninteractive

ENV ROS_DISTRO jazzy
ENV ROS_VERSION 2
ENV ROS_DOMAIN_ID 0
ENV ROS_LOCALHOST_ONLY 1

ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

ENV GZ_VERSION harmonic

ENV CONTAINER_WORKDIR /home/ubuntu/project
ENV WORKSPACE_ROOT="${CONTAINER_WORKDIR}/ros_robotics"
ENV VENV_PATH="${WORKSPACE_ROOT}/venv"

ENV PIP_BREAK_SYSTEM_PACKAGES 1

RUN apt-get update
RUN apt-get install -y curl lsb-release gnupg

RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt-get update
RUN apt-get install -y gz-${GZ_VERSION}

RUN apt-get update && apt-get install -y \
   python3 \
   python3-pip \
   python3-venv \
   libogre-1.12-dev

RUN rosdep update --rosdistro ${ROS_DISTRO}
        
WORKDIR ${CONTAINER_WORKDIR}
VOLUME ${CONTAINER_WORKDIR}

RUN mkdir -p ${WORKSPACE_ROOT}
COPY ./docker/ros_entrypoint.sh .
COPY ./ros_robotics ${WORKSPACE_ROOT}
RUN chmod +x ros_entrypoint.sh && ./ros_entrypoint.sh

COPY ./docker/ros_run.sh ${WORKSPACE_ROOT}
RUN chmod +x ${WORKSPACE_ROOT}/ros_run.sh

# setup entrypoint
#ENTRYPOINT ["/ros_entrypoint.sh"]
#CMD ["bash"]