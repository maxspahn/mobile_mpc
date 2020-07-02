#!/usr/bin/env python

import os
import sys
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)),
'../forcesLib/diffDrive/'))

import rospy
import time
import tf
import numpy as np
import diffDrive_MPC_py
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Float64MultiArray


class MPCController(object):

    def __init__(self):
        rospy.init_node('mpcNode')
        self.pubLeftWheel = rospy.Publisher('/boxer/left_wheel/command', Float64, queue_size=10)
        self.pubRightWheel = rospy.Publisher('/boxer/right_wheel/command', Float64, queue_size=10)
        self.subOdom = rospy.Subscriber('/boxer/ground_truth_odom', Odometry, self.odom_cb)
        self.subGoal = rospy.Subscriber('/boxer/mpc_goal', Float64MultiArray, self.goal_cb)
        self.curU = np.array([0, 0])
        self.curState = np.array([0, 0, 0])
        self.goal = np.array([0, 0, 0])
        time.sleep(1)
        print("MPC Node initialized")
        print("Current Goal : ", self.goal)
        print("Current State : ", self.curState)

    def odom_cb(self, odometry):
        curPos = np.array([odometry.pose.pose.position.x, odometry.pose.pose.position.y])
        curQuat = odometry.pose.pose.orientation
        quaternion = (curQuat.x, curQuat.y, curQuat.z, curQuat.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        curOri = np.array([euler[2]])
        self.curState = np.concatenate((curPos, curOri));

    def goal_cb(self, goalData):
        self.goal = np.array(goalData.data)

    def computeError(self):
        return np.linalg.norm(self.curState[0:2] - self.goal[0:2])

    def solve(self):
        """Solves the MPC problem for the current state

        :curState: actual robot state
        :returns: x_exp expected next state
                  u_opt optimal control input for next timestep

        """
        time_horizon = 15
        dt = 0.1
        wheel_radius = 0.08
        wheel_distance = 0.544
        start = self.curState
        start_u = self.curU
        xinit = np.concatenate((start, start_u))
        x0 = np.tile(xinit, time_horizon)
        setup = np.array([dt, wheel_radius, wheel_distance])
        goal = self.goal
        obstacle = np.array([4.5, 4, 0, 1.5])
        singleParam = np.concatenate((setup, goal, obstacle))
        print("singleParam", singleParam)
        params = np.tile(singleParam, time_horizon)
        PARAMS = {}
        PARAMS['xinit'] = xinit
        PARAMS['x0'] = x0
        PARAMS['all_parameters'] = params
        solution = diffDrive_MPC_py.diffDrive_MPC_solve(PARAMS)
        x02 = solution[0]['x02']
        x_exp = x02[0:3]
        u_opt = x02[3:5]
        return [x_exp, u_opt]

    def publishVelocities(self, u):
        u_right = Float64(u[0])
        u_left = Float64(u[1])
        self.pubRightWheel.publish(u_right)
        self.pubLeftWheel.publish(u_left)

    def singleMPCStep(self):
        [x_exp, u_opt] = self.solve()
        print("state : ", self.curState)
        print("goal : ", self.goal)
        self.curU = u_opt
        self.publishVelocities(u_opt)
        time.sleep(0.1)

    def executeRequest(self):
        error = 100
        while True:
            oldError = error
            error = self.computeError()
            print(error)
            if error < 0.01:
                break
            self.singleMPCStep()
        self.publishVelocities(np.array([0, 0]))

if __name__ == "__main__":
    mpc_cont = MPCController()
    while not rospy.is_shutdown():
        mpc_cont.executeRequest()
        print("Request executed")
        print("Waiting for new Requests, you can use the send_mpc_base_goal script")
        time.sleep(10)
