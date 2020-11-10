### MPC Trajectory Optimization for Mobile Manipulators

## Preliminaries

[MATLAB](https://www.mathworks.com/products/matlab.html)
[forcesPro](https://www.embotech.com/products/forcespro/overview/)
[casadi](https://web.casadi.org/get/)

##External dependencies

[decompUtilRos](https://github.com/sikang/DecompROS)
```
pip install geomdl
```

## Code generation

The underlying optimization problem is solved using FORCESPro.
The corresponding c-code is generated in MATLAB. 
In mobile_mpc/forcesLib/simpleMPC you find `` createSimpleMPC.m ``.
Open this file and fill in the path to your forces folder.
Run this script to generate the code. It might take some minutes.

## Building the ros Node

```
bash
catkin build mobile_mpc
```

## Launching

Setting update frequency of move_base to 0 to avoid mulitple updates of the global path
```
roslaunch mobile_mpc mpc_gazebo.launch
roslaunch mobile_mpc mpc_components.launch
roslaunch mobile_mpc mpc_planner.launch
roscd mobile_mpc/scripts
./simpleMpcActionClient 3.0 0.0 0.0
```
