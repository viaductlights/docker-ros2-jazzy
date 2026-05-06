#!/bin/bash

XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.x11
SHARED_DIR=/root/shared/ros2
HOST_DIR=$HOME/shared/ros2

mkdir -p $HOST_DIR

touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

echo -e "\e[32mMounting folder:
    $HOST_DIR    to
    $SHARED_DIR\e[0m"

docker run \
    -it --rm \
    --name "probrobotics" \
    --privileged \
    --net=host \
    --ipc=host \
    --volume=$XSOCK:$XSOCK:rw \
    --volume=$XAUTH:$XAUTH:rw \
    --volume=$HOST_DIR:$SHARED_DIR:rw \
    --volume=/dev:/dev \
    --env="XAUTHORITY=${XAUTH}" \
    --env="DISPLAY=${DISPLAY}" \
    --env="ROS_DOMAIN_ID=88" \
    probrobotics:latest bash

rm $XAUTH
