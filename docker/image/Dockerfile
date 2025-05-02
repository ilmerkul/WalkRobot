FROM ubuntu:jammy-20250404

ENV DEBIAN_FRONTEND noninteractive

ENV ROS_DISTRO humble
ENV ROS_VERSION 2
ENV ROS_DOMAIN_ID 0
ENV ROS_LOCALHOST_ONLY 1

ENV CONTAINER_WORKDIR /home/ubuntu/project
ENV VENV_PATH="${CONTAINER_WORKDIR}/.venv"

# Install essentials
RUN apt-get update \
	&& apt-get upgrade -y \
	&& apt-get install -y \
		apt-utils \
		# Sound
		libasound2-plugins \
		alsa-utils 

# Setup ROS2 from source as per
RUN apt update && apt install -y locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && export LANG=en_US.UTF-8 \
    && locale
RUN apt-get install software-properties-common -y \
	&& add-apt-repository universe
RUN apt update && apt-get install curl -y \
	&& curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt update \
	&& apt upgrade -y \
	&& apt-get install -y ros-${ROS_DISTRO}-ros-base \
	&& apt-get install -y ros-dev-tools ros-${ROS_DISTRO}-rclpy
RUN apt install -y ros-${ROS_DISTRO}-urdf-launch \
		ros-${ROS_DISTRO}-xacro \
		ros-${ROS_DISTRO}-rqt-robot-steering

# Setup gazebo
RUN apt-get update \
	&& apt-get install -y lsb-release gnupg
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
	&& echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
	&& apt-get update \
	&& apt-get install -y ignition-fortress

RUN apt-get install -y ros-${ROS_DISTRO}-ros-gz \
		ros-${ROS_DISTRO}-ros2-control \
		ros-${ROS_DISTRO}-ros2-controllers \
		ros-${ROS_DISTRO}-gazebo-ros-pkgs \
		ros-${ROS_DISTRO}-urdf-tutorial \
		ros-${ROS_DISTRO}-gazebo-ros2-control

# Install Python, pip, and poetry
RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    python3-venv \
    && rm -rf /var/lib/apt/lists/*

RUN apt install -y python3-colcon-common-extensions

WORKDIR ${CONTAINER_WORKDIR}
COPY ./docker/install.sh /tmp/install.sh
COPY ./poetry.lock .
COPY ./pyproject.toml .
RUN chmod +x /tmp/install.sh
RUN /tmp/install.sh ${VENV_PATH}

VOLUME ${CONTAINER_WORKDIR}

CMD [ "/bin/bash" ]
