#!/bin/bash


#absolute path of this script
SCRIPT_PATH=$(dirname $(realpath "$0"))

#parent path
PARENT_PATH=$(dirname "$SCRIPT_PATH")

#docker build process
build_docker_image()
{

	#log msg
	LOG="building docker image probrobotics:latest ..."
	echo ""
	echo "$LOG"
	echo ""

	sudo docker image build -f $SCRIPT_PATH/Dockerfile -t probrobotics:latest $PARENT_PATH --no-cache
}

#create shared folder
create_shared_folder()
{
	# Check if shared directory exists
	if [ ! -d "$HOME/shared/ros2" ]; then
		# log msg
		LOG="Creating $HOME/shared/ros2 ..."
		echo ""
		echo "$LOG"
		echo ""
		
		# Create the directory and its parent directories if they don't exist
		mkdir -p $HOME/shared/ros2
	fi
}

create_shared_folder
build_docker_image
