### Launch  

Launches RViz2 with Nav2 display active, Turtlebot4 in gz-sim, and all nodes  
`ros2 launch state_estimation sim_bringup_updated.launch.py`  

#### Trajectories  

Trajectory 1: turtlebot position initialized, Nav2 display active. Default with above launch command.  
Trajectory 2: short trajectory using NavigateThroughPoses action server  
Trajectory 3: long trajectory using NavigateThroughPoses action server  
Trajectory 4: short trajectory using NavigateToPose action server  
Trajectory 5: long trajectory using NavigateToPose action server  

specify with
`ros2 launch state_estimation sim_bringup_updated.launch.py test_trajectory_version:=`  

#### Filters

| Filter                    | Node    | Path topic       | Pose topic   | PoseArray topic |
|---------------------------|---------|------------------|--------------|-----------------|
| Kalman Filter             | kf      | tb4_kf_path      | pose_kf      | N/A             |
| EKF Sensor-Fusion         | ekfo    | tb4_ekfo_path    | pose_ekfo    | N/A             |
| EKF Landmarks Gated       | ekf_g   | tb4_ekfg_path    | pose_ekfg    | N/A             |
| Particle Filter           | pf      | tb4_pf_path      | pose_pf      | /particles      |
| KF with quaternion error  | kf_qe   | tb4_kf_qe_path   | pose_kf_qe   | N/A             |
| EKF Sensor-Fusion with qe | ekfo_qe | tb4_ekfo_qe_path | pose_ekfo_qe | N/A             |
| EKF Landmarks Ungated     | ekf_ug  | tb4_ekf_ug_path  | pose_ekf_ug  | N/A             |
| Particle Filter with qe   | pf_qe   | tb4_pf_qe_path   | pose_pf_qe   | /particles_qe   |
| EKF dead reckoning        | N/A     | tb4_ekf_dr_path  | pose_ekf_dr  | N/A             |
