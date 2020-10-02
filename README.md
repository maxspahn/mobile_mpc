### MPC Trajectory Optimization for Mobile Manipulators

##External dependencies

[decompUtilRos](https://github.com/sikang/DecompROS)
[forcesPro](https://www.embotech.com/products/forcespro/overview/)

## Code generation
In the forcesLib folder open the createSimpleMPC.m file and insert your installation path
of forces and casadi.
You can execute this code to generate the solver.

## Building

```
catkin build mobile_mpc
```

## Launching

Setting update frequency of move_base to 0 to avoid mulitple updates of the global path
```
roslaunch mobile_mpc mpc_test.launch
roslaunch mobile_navigation move_base.launch
roslaunch mobile_navigation map_creating.launch
roslaunch mobile_mpc helpers.launch
roslaunch mobile_mpc simpleMpc_planner.launch
roscd mobile_mpc/scripts
./simpleMpcActionClient 0.0 6.0 0.0
```
