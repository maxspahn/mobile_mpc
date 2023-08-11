### MPC Trajectory Optimization for Mobile Manipulators

> [!WARNING]
> This code is not maintained or recently tested

Code for paper "Cupled Mobile Manipulation via Trajectory Optimization with Free
Space Decomposition".

If you are using this repository, please cite the paper:
```bash
@INPROCEEDINGS{9561821,
  author={Spahn, Max and Brito, Bruno and Alonso-Mora, Javier},
  booktitle={2021 IEEE International Conference on Robotics and Automation (ICRA)}, 
  title={Coupled Mobile Manipulation via Trajectory Optimization with Free Space Decomposition}, 
  year={2021},
  volume={},
  number={},
  pages={12759-12765},
  doi={10.1109/ICRA48506.2021.9561821}}

```


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
