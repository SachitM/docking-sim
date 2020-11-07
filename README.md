[![ROS Distro: Melodic](https://img.shields.io/badge/ROS-Melodic-blue.svg)](http://wiki.ros.org/melodic) |license|
.. |license| image:: https://img.shields.io/badge/License-MIT-blue.svg
   :target: http://opensource.org/licenses/MIT
=============
Autonomous Docking for an Ackermann Vehicle
=============

# Team Align

Please refer to [our website](https://mrsdprojects.ri.cmu.edu/2020teamj/system-design/) for more detailed system implementation.

## Setup and Usage
Since this is ROS package, you will need to [install ROS](http://wiki.ros.org/Installation/Ubuntu). This repo has been tested on Ubuntu 18.04 (ROS
Melodic). You will also need Autoware and you can refer to [Autoware Installation](https://github.com/CPFL/Autoware-Manuals/blob/master/en/Autoware_UsersManual_v1.1.md#3-d-map-generation-and-sharing)
Requirements: Ubuntu 18.04+ and ROS-Melodic, python 2.7, move_base, GTSAM and Autoware

`apriltag_ros` depends on the latest release of the [AprilTag library](https://github.com/AprilRobotics/apriltag). Clone it and follow instructions.

For docking simulation, clone the repo in catkin workspace and run 
```
catkin build
source devel/setup.bash
roslaunch align_gazebo align.launch
```
In other terminals (Do not forget to run the env with Python 2.7 and to source ROS)
```
sh run_keyop.sh #To manually drive the vehicle
roslaunch pod_localizer goal_pub.launch #For Localizing Pod
roslaunch align_navigation mapless_move_base.launch #For Running Planner
rosrun align_gazebo pure_pursuit.py #To plan a path till goal
rosrun align_navigation goal_publisher.py #To use other planners
```

For Autoware Simulation
    Replace the ..path-to-autoware/autoware/install/vehicle_model with docking-sim/vehicle_model
    Replace the ..path-to-autoware/autoware/install/vehicle_gazebo_simulation_launcher with docking-sim/vehicle_gazebo_simulation_launcher
Run
```
source install/setup.bash 
#Make sure autoware environment is activated
roslaunch vehicle_gazebo_simulation_launcher gazebo_launcher.launch world_name:=simple gpu:=true
```
Autoware
```
#Run Autoware and Configure for use - Required is path planner, NDT localizer and Rviz
roslaunch runtime_manager runtime_manager.launch
```
Move to docking-sim folder, source and run the following 
```
roslaunch align_gazebo autoware.launch #HMS, Obstacle Detection and PHZ Identification
#HMS TESTS
rosrun hms_client scan_dummy.py
rosnode kill /obstacle_2d
rosrun obstacle_2d obstacle_2d

#Docking
roslaunch align_navigation mapless_move_base.launch
roslaunch pod_localizer goal_pub_autoware.launch #For Localizing Pod
rosrun align_navigation goal_publisher.py #For Approach Navigation

```

Future Updates

To install GTSAM (Latest Version)
```
sudo apt-add-repository ppa:bernd-pfrommer/libgtsam
sudo apt update
sudo apt install libgtsam-unstable4 libgtsam4 libgtsam-dev libgtsam-unstable-dev
```
If Previously installed
```
sudo apt remove gtsam
sudo add-apt-repository --remove ppa:bernd-pfrommer/gtsam
```

## Citation

Please cite outwork if you extend this

##Support
Send a mail to [Sachit Mahajan](mailto:sachitma@andrew.cmu.edu)


References
==========
```
.. [Dubins51] Dubins, L.E. (July 1957). "On Curves of Minimal Length with a Constraint on Average Curvature, and with Prescribed Initial and Terminal Positions and Tangents". American Journal of Mathematics 79 (3): 497–516
.. [LaValle06] LaValle, S. M. (2006). "Planning Algorithms". Cambridge University Press
.. [Shkel01] Shkel, A. M. and Lumelsky, V. (2001). "Classification of the Dubins set". Robotics and Autonomous Systems 34 (2001) 179–202
.. [Walker11] Walker, A. (2011). "Hard Real-Time Motion Planning for Autonomous Vehicles", PhD thesis, Swinburne University.
.. [Smart08] Royce, S. (2008). "Evolutionary Control of Autonomous Underwater Vehicles". PhD thesis, RMIT
.. [Reeds] Reeds, J., & Shepp, L. (1990). Optimal paths for a car that goes both forwards and backwards. Pacific journal of mathematics, 145(2), 367-393.
```
For 'april_tag_ros'
If you use this code, please kindly inform [Danylo Malyuta](mailto:danylo.malyuta@gmail.com) (to maintain a list here of research works that have benefited from the code) and cite:

- D. Malyuta, C. Brommer, D. Hentzen, T. Stastny, R. Siegwart, and R. Brockers, “[Long-duration fully autonomous operation of rotorcraft unmanned aerial systems for remote-sensing data acquisition](https://onlinelibrary.wiley.com/doi/abs/10.1002/rob.21898),” Journal of Field Robotics, p. arXiv:1908.06381, Aug. 2019.
- C. Brommer, D. Malyuta, D. Hentzen, and R. Brockers, “[Long-duration autonomy for small rotorcraft UAS including recharging](https://ieeexplore.ieee.org/document/8594111),” in IEEE/RSJ International Conference on Intelligent Robots and Systems, IEEE, p. arXiv:1810.05683, oct 2018.
- J. Wang and E. Olson, "[AprilTag 2: Efficient and robust fiducial detection](http://ieeexplore.ieee.org/document/7759617/)," in ''Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)'', October 2016.


.. |license| image:: https://img.shields.io/badge/License-MIT-blue.svg
   :target: http://opensource.org/licenses/MIT
   :alt: license shield