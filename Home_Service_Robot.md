# Home Service Robot



## Introduction
**Function**: 
The home service robot should be simulated as follow:

Initially show the marker at the pickup zone, hide the marker once your robot reach the pickup zone, then wait 5 seconds to simulate a pickup and finally show the marker at the drop off zone once the robot reaches it.

**Software components**

To implement the robot function, the following packages are created.

* Package "add_markers": provides rosservice "DealMarkerAt", which can add or delete the marker at the given place.

* Package "my_robot": constructs robot and environment model, invoke amcl package for the localization.

* Package "pick_objects": plans the path to the goal using move_base and calls rosservice "DealMarkerAt" to show or hide the marker.


**Required package**

* [amcl](http://wiki.ros.org/amcl):  Probabilistic localization system for a robot moving in 2D using particle filter. 

* [slam_gammping](http://wiki.ros.org/gmapping): A ROS wrapper for OpenSlam's Gmapping. (This is just used  to run test_slam.sh script file to manually test SLAM.)

* [move_base](http://wiki.ros.org/move_base): The move_base node links together a global and local planner to accomplish a global navigation task. 

**Quick start**

Use file "home_service.sh" file to run all the nodes.

