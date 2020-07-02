import numpy as np
import mm_MPC_py


time_horizon = 15
start = np.array([0, 0, 1, 0, 0, 0, -1, 0.5, 1.5, 0])
start_u = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0])
xinit = np.concatenate((start, start_u))
x0 = np.tile(xinit, time_horizon)
setup = np.array([0.01, 0.1, 0.5])
goal = start
goal[5] = 1
obstacle = np.array([30, 30, 30, 0.1])
singleParam = np.concatenate((setup, goal, obstacle))
params = np.tile(singleParam, time_horizon)
PARAMS = {}
PARAMS['xinit'] = xinit
PARAMS['x0'] = x0
PARAMS['all_parameters'] = params


a = mm_MPC_py.mm_MPC_solve(PARAMS)
print(a[0]['x01'])
