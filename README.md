## Probabilistic Robotics Lab Project with ROS2 and Turtlebot4  

#### WHAT STARTS BADLY CAN NEVER IMPROVE... UNLESS: Measurement Model Error in Probabilistic State Estimation

[Project README Page](https://github.com/viaductlights/docker-ros2-jazzy/blob/main/src/state_estimation/README.md)


##### docker container for probabilistic robotics lab project  
runs ros2-jazzy on bazzite linux with turtlebot4 and other misc useful ros packages pre installed. includes dummy ros2 package called "robot_modelling" for a project in another class

###### build docker container for the first time  
`bash build.sh`  

###### run docker container once built  
`bash docker_run.sh`  

###### switch to bazzite-dx for docker/vscode  
check current system, rebase if not already bazzite-dx  
```shell
rpm-ostree status  
brh rebase bazzite-dx:stable  
systemctl reboot  
```
check if docker group has been created, and if not (due to known ublue-os ujust dx-group bug as of 4.2026), add docker group and local user   
```shell
groups  
sudo groupadd docker  
sudo usermod -aG docker $USER  
systemctl reboot  
ujust dx-group  
systemctl reboot  
```

test  
`docker run hello-world`  

