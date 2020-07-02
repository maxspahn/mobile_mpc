import numpy as np
import mm_MPC_py
import time


time_horizon = 12
dt = 0.5
start = np.array([0, 0, 0, 0, 0, 0, 0, 0, -1, 0.5, 1.5, 0])
start_slack = np.zeros(1)
start_u = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0])
xinit = np.concatenate((start, start_slack, start_u))
x0 = np.tile(xinit, time_horizon)
setup = np.array([dt, 0.1, 0.5])
goal = np.zeros(10)
goal[5] = 1
goal[9] = -1
goal[0] = 5
obstacles = np.tile(np.array([-100, -100, -100, -100]), 50)
obstacles = np.ones(50 * 4) * -100
weights = [1, 1, 0, 1, 0, 0, 10]
singleParam = np.concatenate((setup, goal, weights, obstacles))
params = np.tile(singleParam, time_horizon)
PARAMS = {}
PARAMS['xinit'] = xinit
PARAMS['x0'] = x0
PARAMS['all_parameters'] = params
print(len(params))


while True:
    [output, exitFlag, info] = mm_MPC_py.mm_MPC_solve(PARAMS)
    curState = output['x02']
    PARAMS['xinit'] = curState
    PARAMS['x0'] = np.tile(curState, 12)
    time.sleep(0.01)
    print("Costs : ", info.pobj)
    print(exitFlag)
    print(curState[0:2])
    print("slack variable : ", curState[10])
    if np.linalg.norm(curState[0:2] - goal[0:2]) < 0.1:
        break

