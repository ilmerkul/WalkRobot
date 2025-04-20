FROM ubuntu:jammy-20250404


ENV ROS_DISTRIBUTION humble
ENV CONTAINER_WORKDIR /project
ENV DEBIAN_FRONTEND noninteractive


# Install essentials
RUN \
	apt-get update \
	&& apt-get upgrade -y \
	&& apt-get install -y \
		apt-utils

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
	&& apt-get install -y ros-${ROS_DISTRIBUTION}-ros-base \
	&& apt-get install -y ros-dev-tools ros-${ROS_DISTRIBUTION}-rclpy

# Setup gazebo
RUN apt-get update \
	&& apt-get install -y lsb-release gnupg
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
	&& echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
	&& apt-get update \
	&& apt-get install -y ignition-fortress
RUN apt-get install -y ros-${ROS_DISTRIBUTION}-ros-gz

# Install Python, pip, and poetry
RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    python3-venv \
	python3-poetry \
    && rm -rf /var/lib/apt/lists/*

RUN apt install -y python3-colcon-common-extensions

WORKDIR ${CONTAINER_WORKDIR}
VOLUME ${CONTAINER_WORKDIR}

CMD ["source", "/opt/ros/${ROS_DISTRIBUTION}/setup.bash"]
