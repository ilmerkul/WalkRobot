#!/bin/bash

# Скрипт для запуска Docker-образа
set -e  # Прерывать выполнение при ошибках

# Проверка обязательных переменных окружения
if [ -z "$PROJECT_DIR" ] || [ -z "$CONTAINER_WORKDIR" ] || [ -z "$DOCKER_IMAGE" ]; then
  echo "Ошибка: Необходимо задать переменные окружения:"
  echo "  PROJECT_DIR, CONTAINER_WORKDIR, DOCKER_IMAGE"
  exit 1
fi

X11_SUPPORT=$([ -n "$DISPLAY" ] && echo "--env=DISPLAY --env=QT_X11_NO_MITSHM=1 --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw")

# Запуск контейнера
docker run -it \
  --env CONTAINER_WORKDIR=$CONTAINER_WORKDIR \
  --env ROS_DISTRO=$ROS_DISTRO \
  --env ROS_VERSION=$ROS_VERSION \
  --env ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
  --env ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY \
  --volume="$PROJECT_DIR/ros_robotics:$CONTAINER_WORKDIR/ros_robotics" \
  --volume="$PROJECT_DIR/.env:$CONTAINER_WORKDIR/.env" \
  $X11_SUPPORT \
  "$DOCKER_IMAGE" bash