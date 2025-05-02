#!/bin/bash

# Скрипт для запуска Docker-образа
set -e  # Прерывать выполнение при ошибках

# Проверка обязательных переменных окружения
if [ -z "$PROJECT_DIR" ] || [ -z "$CONTAINER_WORKDIR" ] || [ -z "$DOCKER_IMAGE" ] || [ -z "$ROS_DISTRO" ] || [ -z "$ROS_VERSION" ] || [ -z "$ROS_DOMAIN_ID" ] || [ -z "$ROS_LOCALHOST_ONLY" ]; then
  echo "Ошибка: Необходимо задать переменные окружения:"
  echo "  PROJECT_DIR, CONTAINER_WORKDIR, DOCKER_IMAGE, ROS_DISTRO, ROS_VERSION, ROS_DOMAIN_ID, ROS_LOCALHOST_ONLY"
  exit 1
fi

# Запуск контейнера
docker run -it \
  --env CONTAINER_WORKDIR=$CONTAINER_WORKDIR \
  --env ROS_DISTRO=$ROS_DISTRO \
  --env ROS_VERSION=$ROS_VERSION \
  --env ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
  --env ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY \
  --volume="$PROJECT_DIR/dl:$CONTAINER_WORKDIR/dl" \
  --volume="$PROJECT_DIR/ros_robotics:$CONTAINER_WORKDIR/ros_robotics" \
  --volume="$PROJECT_DIR/poetry.lock:$CONTAINER_WORKDIR/poetry.lock" \
  --volume="$PROJECT_DIR/pyproject.toml:$CONTAINER_WORKDIR/pyproject.toml" \
  --volume="$PROJECT_DIR/.env:$CONTAINER_WORKDIR/.env" \
  "$DOCKER_IMAGE"
