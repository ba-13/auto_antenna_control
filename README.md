# EE798T Course Project: Automatic Antenna Control

## Install prerequisites

```bash
./install_tic.sh
```

## Build

Entire project is in the `workspace` folder. 
Ensure you have ROS Noetic Full installed, and have Python3.8

```
cd workspace
catkin init

catkin build
```

## Run

Correct configuration parameters from [here](./workspace/src/messages/config/).
Now `source devel/setup.bash` and run

```
roslaunch initialization initialization.launch
```

Let this align the pitch/elevation of the antenna and set bias to 0.

Now run:

```
roslaunch antenna_control motors.launch
```

This should start making the antenna move according to the simulator of drone pose present [here](./workspace/src/odom_simulate/src/drone_sim_node.cpp)
