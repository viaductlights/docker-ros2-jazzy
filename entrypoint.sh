#!/bin/bash
set -e

#ROS distro
ROS_DISTRO=${ROS_DISTRO:-jazzy}
OVERLAY_WS=${OVERLAY_WS:-/opt/ros/overlay_ws}

#temp director for GUI
#export XDG_RUNTIME_DIR=/tmp/runtime-$USER
#mkdir -p $XDG_RUNTIME_DIR
#chmod 700 $XDG_RUNTIME_DIR

#base path
#OVERLAY_WS="/opt/ros/overlay_ws"

#shared directory path
#SHARED_ROS2="/root/shared/ros2"

#networking
if [ -z "${ROS_DOMAIN_ID}" ]; then
    export ROS_DOMAIN_ID=88
fi
echo "ROS_DOMAIN_ID set to: ${ROS_DOMAIN_ID}"

#gui
if [ -z "${XDG_RUNTIME_DIR}" ]; then
    export XDG_RUNTIME_DIR=/tmp/runtime-root
    mkdir -p $XDG_RUNTIME_DIR
    chmod 0700 $XDG_RUNTIME_DIR
fi
echo "XDG_RUNTIME_DIR set to : ${XDG_RUNTIME_DIR}"


#domain_id.txt for ROS_DOMAIN_ID
#ROS_DOMAIN_ID_FILE="$SHARED_ROS2/ros_domain_id.txt"

#if [ ! -f "$ROS_DOMAIN_ID_FILE" ]; then
	#replace non existent file w 0
#	echo "0" > "ROS_DOMAIN_ID_FILE"
#	echo "Created $ROS_DOMAIN_ID_FILE with default value 0"
#fi

#update and export ROS_DOMAIN_ID
#if ! grep -q "export ROS_DOMAIN_ID" /root/.bashrc; then
#    ros_domain_id=$(cat "$ROS_DOMAIN_ID_FILE")
#    echo "export ROS_DOMAIN_ID=$ros_domain_id" >> /root/.bashrc
#fi

#export ROS_DOMAIN_ID=$(cat "$ROS_DOMAIN_ID_FILE")

#source base ROS setup file
source /opt/ros/$ROS_DISTRO/setup.bash
echo "sourced underlay from /opt/ros/$ROS_DISTRO"

#source overlay workspace
if [ -f "$OVERLAY_WS/install/setup.bash" ]; then
    source $OVERLAY_WS/install/setup.bash
    echo "Sourced overlay workspace from $OVERLAY_WS"
else
    echo "WARNING: Overlay workspace not found at $OVERLAY_WS. Only base ROS is available."
fi

exec "$@"

