## docker container for probabilistic robotics lab project  
runs ros2-jazzy on bazzite linux with turtlebot4 and other misc useful ros packages pre installed. includes dummy ros2 package called "robot_modelling" for a project in another class

### switch to bazzite-dx for docker/vscode  
check current system, rebase if not already bazzite-dx  
`rpm-ostree status`  
`brh rebase bazzite-dx:stable`  
`systemctl reboot`  

check if docker group has been created, and if not (due to known ublue-os ujust dx-group bug as of 4.2026), add docker group and local user   
`groups`  
`sudo groupadd docker`  
`sudo usermod -aG docker $USER`  
`systemctl reboot`  
`ujust dx-group`  
`systemctl reboot`  

test  
`docker run hello-world`  

### build docker container for the first time  
`./build.sh`  

### run docker container once built  
`./docker_run.sh`  

### new bash shell for running container  
`docker exec -it probrobotics bash`  
