#!/bin/bash


#absolute path of this script
SCRIPT_PATH="$(cd "$(dirname "$0")" && pwd)"

#parent path
PARENT_PATH="$(dirname "$SCRIPT_PATH")"

#docker build process
build_docker_image()
{

	#log msg
	LOG="building docker image probrobotics:latest ..."
	echo ""
	echo "$LOG"
	echo ""
	
	echo "SCRIPT_PATH=$SCRIPT_PATH"
	echo "PARENT_PATH=$PARENT_PATH"

	sudo docker image build \
	    -f "$SCRIPT_PATH/Dockerfile" \
	    -t probrobotics:latest \
	    --no-cache \
	    "$PARENT_PATH"
}

#create shared folder
create_shared_folder()
{
	# Check if shared directory exists
	if [ ! -d "$PARENT_PATH/src" ]; then
		# log msg
		LOG="Creating $PARENT_PATH/src ..."
		echo ""
		echo "$LOG"
		echo ""
		
		# Create the directory and its parent directories if they don't exist
		mkdir -p "$PARENT_PATH/src"
	fi
}

create_shared_folder
build_docker_image
