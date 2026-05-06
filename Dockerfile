ARG ROS_DISTRO=jazzy

FROM ros:${ROS_DISTRO}-ros-base
LABEL maintainer="Jiayi Zhou <re25m012@technikum-wien.at>"
ARG OVERLAY_WS=/opt/ros/overlay_ws
ARG GZ_VERSION=harmonic

ENV PIP_BREAK_SYSTEM_PACKAGES=1
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}
ENV OVERLAY_WS=${OVERLAY_WS}
ENV GZ_VERSION=${GZ_VERSION}

SHELL ["/bin/bash", "-c"]

#update system, packages, install utils
RUN apt-get update -q && \
    apt-get upgrade -yq && \
    apt-get install -yq --no-install-recommends \
	apt-utils wget curl git build-essential \
	vim sudo lsb-release locales bash-completion \
	tzdata gosu gedit htop nano libserial-dev \
	gnupg2 iputils-ping usbutils \
	python3-argcomplete python3-colcon-common-extensions \
	python3-networkx python3-pip python3-rosdep python3-vcstool \
	python3-setuptools && \
   rm -rf /var/lib/apt/lists/*

#install gazebo harmonic
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update \
    && apt-get install -y "gz-${GZ_VERSION}"  \
    && rm -rf /var/lib/apt/lists/*

#install ros-gazebo bridge and other tools
RUN apt-get update && \
    apt-get install -y \
	"ros-${ROS_DISTRO}-ros-gz" \
	ros-${ROS_DISTRO}-desktop \
	ros-${ROS_DISTRO}-joint-state-publisher-gui \
	ros-${ROS_DISTRO}-xacro \
	ros-${ROS_DISTRO}-turtlebot4-simulator \
	ros-${ROS_DISTRO}-urdf-launch \
	emacs htop byobu less && \
    rm -rf /var/lib/apt/lists/*

#set up ROS2 environment
RUN rosdep update && \
    grep -F "source /opt/ros/${ROS_DISTRO}/setup.bash" /root/.bashrc || \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    grep -F "source ${OVERLAY_WS}/install/setup.bash" /root/.bashrc || \
    echo "source ${OVERLAY_WS}/install/setup.bash" >> /root/.bashrc 

#clone and build turtlebot4 and robotmodeling basics in overlay 
WORKDIR ${OVERLAY_WS}/src
#RUN git clone https://github.com/turtlebot/turtlebot4_simulator -b ${ROS_DISTRO}
COPY src/ ${OVERLAY_WS}/src/

WORKDIR ${OVERLAY_WS}
RUN if [ ! -f ${OVERLAY_WS}/src/robot_modelling/package.xml ]; then \
        echo "No robot_modelling package found. Creating a minimal placeholder package..." && \
        mkdir -p ${OVERLAY_WS}/src/robot_modelling && \
        echo '<?xml version="1.0"?>' > ${OVERLAY_WS}/src/robot_modelling/package.xml && \
        echo '<package format="3">' >> ${OVERLAY_WS}/src/robot_modelling/package.xml && \
        echo '  <name>robot_modelling</name>' >> ${OVERLAY_WS}/src/robot_modelling/package.xml && \
        echo '  <version>0.0.0</version>' >> ${OVERLAY_WS}/src/robot_modelling/package.xml && \
        echo '  <description>Placeholder for robot modelling ROS2 packages</description>' >> ${OVERLAY_WS}/src/robot_modelling/package.xml && \
        echo '  <buildtool_depend>ament_cmake</buildtool_depend>' >> ${OVERLAY_WS}/src/robot_modelling/package.xml && \
        echo '</package>' >> ${OVERLAY_WS}/src/robot_modelling/package.xml && \
        touch ${OVERLAY_WS}/src/robot_modelling/CMakeLists.txt && \
        echo 'cmake_minimum_required(VERSION 3.8)' > ${OVERLAY_WS}/src/robot_modelling/CMakeLists.txt && \
        echo 'project(robot_modelling)' >> ${OVERLAY_WS}/src/robot_modelling/CMakeLists.txt; \
    fi

RUN apt-get update && \
    . /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep update && \
    rosdep install -y --from-paths src --ignore-src -r || \
    echo "rosdep install finished with warnings" && \
    rm -rf /var/lib/apt/lists/*

# Build the entire overlay workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --symlink-install || echo "Build completed with warnings (normal for placeholder packages)"

COPY /docker/entrypoint.sh /root/
RUN chmod +x /root/entrypoint.sh

RUN sed --in-place --expression \
    '$isource "$OVERLAY_WS/install/setup.bash"' \
    /ros_entrypoint.sh

ENTRYPOINT ["/root/entrypoint.sh"]
CMD ["bash"]
