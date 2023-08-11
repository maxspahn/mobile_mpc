### MPC Trajectory Optimization for Mobile Manipulators

## Preliminaries

[MATLAB](https://www.mathworks.com/products/matlab.html)
[forcesPro](https://www.embotech.com/products/forcespro/overview/)
optional:
[casadi](https://web.casadi.org/get/)

##External dependencies

[decompUtilRos](https://github.com/sikang/DecompROS)
[ros_mm](https://github.com/maxspahn/ros_mm)
```
pip install geomdl
```

## Code generation

The underlying optimization problem is solved using FORCESPro.
The corresponding c-code is generated in MATLAB. 
First, the path to your forces pro needs to be defined in the file ``addCustomPaths()``.
You can also specify the path to your casadi installation. This is optional as matlab will
automatically install a casadi version if this is not specified.
In mobile_mpc/forcesLib/simpleMPC you find `` createSimpleMPC.m ``.
This script must be executed once from exactly this folder.
In mobile_mpc/forcesLib/sphereMPC you find `` createSphereMPC.m ``.
This script must be executed once from exactly this folder.


## Building the ros Node

```
bash
catkin build mobile_mpc
```
You might build it twice, due to the message generation.

## Launching

Setting update frequency of move_base to 0 to avoid mulitple updates of the global path
```
roslaunch mobile_mpc mpc_gazebo.launch
roslaunch mobile_mpc mpc_components.launch
roslaunch mobile_mpc mpc_planner.launch
roscd mobile_mpc/scripts
./simpleMpcActionClient 3.0 0.0 0.0
```
