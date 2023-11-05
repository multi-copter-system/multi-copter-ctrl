# multi_copter_ctrl

This package provides functions for this project: https://github.com/multi-copter-system/multi-copter-gnc


## How to run

Change directory to src under your ROS workspace.  
Ex. `cd ~/catkin_ws/src`

Clone repositorys.

```
git clone https://github.com/multi-copter-system/multi-copter-ctrl.git
```
```
git clone https://github.com/multi-copter-system/multi-copter-msgs.git
```

Build packages.

```
catkin build
```

After run `catkin build`, you must run the below command:

```
source ~/catkin_ws/devel/setup.bash
```

You can run the below command. Don't forget `roscore`!

```
rosrun multi_copter_ctrl control_server.py
```
```
rosrun multi_copter_ctrl control_client.py
```
