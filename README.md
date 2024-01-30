# Multi Robot ROS Simulator

---
## Table of content
1. [Instructions](#instructions)

---

# Instructions

COMPILATION:
In bash nr. 1:
```code
cd ./rp_ws
source /opt/ros/noetic/setup.bash
catkin_make
source ./devel/setup.bash
```

RUN:
In bash nr. 2:
```code
source /opt/ros/noetic/setup.bash
roscore
```

In bash nr. 3:
```code
cd ./exam
source /opt/ros/noetic/setup.bash
rosrun rviz rviz -d ./config/rviz_basic_config.rviz
```

In bash nr. 4:
```code
source /opt/ros/noetic/setup.bash
source ./devel/setup.bash
rosrun mrsim mover_node NUM_ROBOTS
```

In bash nr. 1:
```code
rosrun mrsim mrsim_node config.json
```
