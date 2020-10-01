#!/usr/bin/env python

import rospy
import numpy as np
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Float64MultiArray, Bool
from sensor_msgs.msg import JointState
from mm_msgs.msg import ActiveInferenceGoal
from nav_msgs.msg import Path

from geomdl import BSpline
from geomdl import utilities
from geomdl.visualization import VisMPL


import numpy as np
import matplotlib.pyplot as plt


class PlanListener(object):
    def __init__(self):
        rospy.init_node("planListener")
        self.rate = rospy.Rate(10)
        self.planSub_ = rospy.Subscriber("/move_base/NavfnROS/plan", Path, self.plan_cb)
        self.plotSub_ = rospy.Subscriber("/nurbs/plot", Bool, self.plot_cb)
        self.evalPub_ = rospy.Publisher(
            "/nurbs/evaluation", Float64MultiArray, queue_size=10
        )
        self.curve = BSpline.Curve()
        self.curve.degree = 3
        self.curve.delta = 0.01
        self.curve.vis = VisMPL.VisCurve2D()
        self.startTime = 0.0
        self.endTime = 0.0

    def plot_cb(self, data):
        print(self.startTime)
        print(self.endTime)
        print(rospy.Time.now())
        t = (rospy.Time.now() - self.startTime) / (self.endTime - self.startTime)
        print(t)
        a = self.curve.evaluate_single(t)
        print(a)
        # self.curve.render()
        self.plotSpline(t)

    def plotSpline(self, evalTime):
        ctrlpts = np.array(self.curve.ctrlpts)
        curvepts = np.array(self.curve.evalpts)
        a = self.curve.evaluate_single(evalTime)

        # Convert tangent list into a NumPy array
        # ctarr = np.array(curvetan)

        # Plot using Matplotlib
        plt.figure(figsize=(10.67, 8), dpi=96)
        yaxis = plt.plot((-1, 25), (0, 0), "k-")  # y-axis line
        (cppolygon,) = plt.plot(
            ctrlpts[:, 0],
            ctrlpts[:, 1],
            color="black",
            linestyle="-.",
            marker="o",
            markersize="3",
        )  # control points polygon
        (curveplt,) = plt.plot(
            curvepts[:, 0], curvepts[:, 1], color="green", linestyle="-"
        )  # evaluated curve points
        plt.plot(a[0], a[1], "ro")
        # tanline = plt.quiver(ctarr[:, 0, 0], ctarr[:, 0, 1], ctarr[:, 1, 0], ctarr[:, 1, 1], color='blue', angles='xy', scale_units='xy', scale=1, width=0.003)  # tangents
        # tanlinekey = plt.quiverkey(tanline, 23.75, -14.5, 35, "Tangent Vectors", coordinates='data', labelpos='W')
        plt.legend([cppolygon, curveplt], ["Control Points", "Evaluated Curve"])
        plt.axis([-5, 5, -5, 5])
        plt.show()

    def plan_cb(self, data):
        rospy.loginfo("Received path")
        rospy.loginfo("Frame : %s", data.header.frame_id)
        rospy.loginfo("Size path : %d", len(data.poses))
        pts = []
        self.startTime = data.poses[0].header.stamp
        self.endTime = data.poses[0].header.stamp
        redPoses = data.poses[0::10]
        for poseStamped in redPoses:
            p = poseStamped.pose
            pts.append([p.position.x, p.position.y])
        if len(pts) > self.curve.degree + 3:
            self.curve.ctrlpts = pts
            self.curve.knotvector = utilities.generate_knot_vector(
                self.curve.degree, len(self.curve.ctrlpts)
            )
            self.startTime = data.poses[0].header.stamp
            self.endTime = data.poses[0].header.stamp + rospy.Duration(
                len(data.poses) / 18
            )
            print(self.endTime)
        else:
            rospy.WARN("Received path to short. Keeping old one")

    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()


if __name__ == "__main__":
    pl = PlanListener()
    pl.run()
