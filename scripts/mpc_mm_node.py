#!/usr/bin/env python

import rospy
import time
import tf
import numpy as np
from mobile_mpc import solve_MPC_mm
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Float64MultiArray
from sensor_msgs.msg import JointState


class MPCController(object):
    def __init__(self):
        print("Initialization of MPC node")
        rospy.init_node("mpcNode")
        self.pubLeftWheel = rospy.Publisher(
            "/mmrobot/left_wheel/command", Float64, queue_size=10
        )
        self.pubRightWheel = rospy.Publisher(
            "/mmrobot/right_wheel/command", Float64, queue_size=10
        )
        self.pubArm = rospy.Publisher(
            "/mmrobot/multijoint_command", Float64MultiArray, queue_size=10
        )
        self.subOdom = rospy.Subscriber(
            "/mmrobot/ground_truth_odom", Odometry, self.odom_cb
        )
        self.subJointPosition = rospy.Subscriber(
            "/mmrobot/joint_states", JointState, self.jointState_cb
        )
        self.subGoal = rospy.Subscriber(
            "/mmrobot/mpc_goal", Float64MultiArray, self.goal_cb
        )
        self.curU = np.zeros(9)
        self.curState = np.zeros(10)
        self.goal = np.zeros(10)
        ex1Goal = np.array([9, 9, -1, 1, 1, 0, -1, 0, 2.5, 2])
        ex2Goal = np.zeros(10)
        ex2Goal[6] = -1
        ex2Goal[8] = 2
        ex3Goal = np.array([9, 9, -1, 1, 1, 0, -1, 0, 2.5, 2])
        self.goal = self.curState
        self.time_horizon = 15
        self.PARAMS = {}
        self.dt = 0.10
        self.problemSetup()
        time.sleep(1)
        print("MPC Node initialized")

    def odom_cb(self, odometry):
        curPos = np.array(
            [odometry.pose.pose.position.x, odometry.pose.pose.position.y]
        )
        curQuat = odometry.pose.pose.orientation
        quaternion = (curQuat.x, curQuat.y, curQuat.z, curQuat.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        curOri = np.array([euler[2]])
        self.curState[0:3] = np.concatenate((curPos, curOri))

    def jointState_cb(self, jointStates):
        self.curState[3:11] = np.array(jointStates.position[3:10])
        self.curU[2:10] = np.array(jointStates.velocity[3:10])

    def goal_cb(self, goalData):
        print("Goal received")
        self.goal = np.array(goalData.data)

    def computeError(self):
        return np.linalg.norm(
            self.curState[0:2] - self.goal[0:2]
        ) + 0.5 * np.linalg.norm(self.curState[3:10] - self.goal[3:10])

    def problemSetup(self):
        wheel_radius = 0.08
        wheel_distance = 0.544
        self.genObstacles()
        self.setup = np.array([self.dt, wheel_radius, wheel_distance])

    def genObstacles(self):
        o0 = np.array([5, 5, 2, 1.5])
        o1 = np.array([1, 2, 2, 1.5])
        o2 = np.array([-0.5, 3.5, 2, 1.5])
        o3 = np.array([-2, 5, 2, 1.5])
        o4 = np.array([-3.5, 6.5, 2, 1.5])
        o5 = np.array([-5, 6.5, 2, 1.5])
        o6 = np.array([4, -2, 1, 1.5])
        o7 = np.array([4, -3.5, 1, 1.5])
        o8 = np.array([4, -5, 1, 1.5])
        o9 = np.array([4, -6.5, 1, 1.5])
        o10 = np.array([5.5, -6.5, 1, 1.5])
        self.obstacles = np.tile(np.concatenate((o0, o1, o2, o3, o4, o6, o7, o8, o9, o10)), 5)

    def solve(self):
        """Solves the MPC problem for the current state

        :curState: actual robot state
        :returns: x_exp expected next state
                  u_opt optimal control input for next timestep

        """
        xinit = np.concatenate((self.curState, self.curU))
        x0 = np.tile(xinit, self.time_horizon)
        singleParam = np.concatenate((self.setup, self.goal, self.obstacles))
        print("Goal : ", self.goal)
        print("CurState : ", self.curState)
        print("Difference : ", self.goal - self.curState)
        params = np.tile(singleParam, self.time_horizon)
        self.PARAMS["xinit"] = xinit
        self.PARAMS["x0"] = x0
        self.PARAMS["all_parameters"] = params
        solution = solve_MPC_mm.solve_MPC_mm(self.PARAMS)
        x02 = solution[0]["x02"]
        x_exp = x02[0:10]
        u_opt = x02[10:19]
        return [x_exp, u_opt]

    def publishVelocities(self, u):
        u_right = Float64(u[0])
        u_left = Float64(u[1])
        u_joints = Float64MultiArray()
        u_joints.data = []
        for i in range(7):
            u_joints.data.insert(i, u[i + 2])
        self.pubRightWheel.publish(u_right)
        self.pubLeftWheel.publish(u_left)
        self.pubArm.publish(u_joints)

    def singleMPCStep(self):
        [x_exp, u_opt] = self.solve()
        self.curU = u_opt
        self.publishVelocities(u_opt)
        time.sleep(self.dt)

    def executeRequest(self):
        error = 100
        while True:
            oldError = error
            error = self.computeError()
            print(error)
            if error < 0.3:
                break
            self.singleMPCStep()
        self.publishVelocities(np.zeros(9))


if __name__ == "__main__":
    mpc_cont = MPCController()
    while not rospy.is_shutdown():
        mpc_cont.executeRequest()
        print("Request executed")
        print("Waiting for new Requests, you can use the send_mpc_mm_goal script")
        time.sleep(10)
