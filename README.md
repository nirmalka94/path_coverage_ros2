# Path Coverage for ROS2

Path coverage is needed for applications like cleaning or mowing where an environment must be fully covered by a robot.
This ROS package executes a coverage path for a given area. The area to cover is given by a polygon which points are set from RViz with "Publish Point". When a successive point equals the first point the area of the polygon is 
divided into cells by an algorithm which resembles the output of the Boustrophedon Cellular Decomposition, so each cell can be covered with simple back and forth motions. The resulting goal points are then given to the navigation stack:

![path coverage demonstration](https://gitlab.com/Humpelstilzchen/path_coverage_ros/-/raw/master/images/path_coverage.gif)

## Requirements
- ROS2 humble or galactic "tested on humble"
- python-shapely
- python-numpy
- ruby for the Boustrophedon Decomposition: $ sudo apt-get install ruby-full

## Usage
1. Start the navigation stack for your robot with move\_base
2. Launch path coverage:

TERMINAL-ROS2: source /opt/ros/humble/setup.bash; source ros2_ws/install/setup.bash; ros2 launch path_coverage path_coverage.launch.py

3. Open RViz, add a *Marker plugin* and set the topic to "path\_coverage\_marker"
4. On the map in RViz, think of a region that you like the robot to cover
5. Click *Publish Point* at the top of RViz
6. Click a single corner of n corners of the region
7. Repeat step 5 and 6 above for n times. After that you'll see a polygon with n corners.
8. The position of the final point should be close to the first
9. When the closing point is detected the robot starts to cover the area

## ROS Nodes
### path\_coverage\_node.py
The node that executes the Boustrophedon Decomposition, calculates the back and forth motions and sends them to move\_base.

#### Subscribed Topics
* "/clicked\_point" - Clicked point from RViz
* "global\_costmap/costmap" - To detect obstacles in path
* "local\_costmap/costmap" - To detect obstacles in path

#### Output
* "pose_output.yaml" - in the home directory. contains waypoints.


### Parameters
* boustrophedon\_decomposition (bool, default: true)

> Whether to execute the Boustrophedon Cellular Decomposition or just do back and forth motions.

* border\_drive (bool, default: false)

> Whether to drive around the cell first before doing back and forth motions.

* robot\_width (float, default: 0.3)

> Width of each path

* costmap\_max\_non\_lethal (float, default: 70)

> Maximum costmap value to consider free

* base\_frame (string, default: "base\_link")

> The robots base frame

* global\_frame (string, default: "map")

> The robots global frame



## Author
ROS1: Erik Andresen - erik@vontaene.de
ROS2:  Azeez Adebayo - hazeezadebayo@gmail.com

## TODO
* The areas to cover needs to be ordered more intelligently (travelling salesman problem)
* Check global planner performance improvement with https://discourse.ros.org/t/global-planner-goal-tolerance/19013
