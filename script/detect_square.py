#!/usr/bin/env python
import rospy 
import rospkg
import csv
import matplotlib.pyplot as plt
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point
from vase_icp.msg import Projected
from math import *
import numpy as np
from scipy import stats
from scipy import optimize
from time import time

rospy.init_node("detect_square", anonymous = True)
pub = rospy.Publisher("s_est_pointcloud", Point, queue_size = 10)



def callback(msg):
    x1 = np.array(msg.x_array.data)
    x2 = np.array(msg.y_array.data)
    x = np.vstack((x1, x2))

    s_est_ = estimate(x)
    s_est = Point(x = s_est_[0], y = s_est_[1], z = s_est_[2])
    pub.publish(s_est)

def evaluate(mu, angle, x_data, fn):
    N = x_data.shape[1]
    x_err = x_data - np.tile(mu, [N, 1]).T
    direction = np.array([cos(angle), sin(angle)])
    tmp = np.tile(direction, [N, 1]).T * (x_err)
    x_projected = np.sum(tmp, axis = 0)
    return fn(x_projected)

def estimate(x_data):
    mu = np.mean(x_data, axis = 1)
    def fn(angle):
        return evaluate(mu, angle, x_data, lambda x: stats.skew(x))
    ret = optimize.minimize(fn, 0, method = 'Nelder-Mead')
    angle_est = ret.x.item()
    std_1 = evaluate(mu, angle_est, x_data, lambda x: np.std(x))
    std_2 = evaluate(mu, angle_est + pi/4 , x_data, lambda x: np.std(x))
    angle_est = (angle_est if std_1 < std_2 else angle_est + pi/4)

    angle_est = angle_est % (pi * 0.5)

    s_est = np.array([mu[0], mu[1], angle_est])

    return s_est

def show():
    global x
    plt.scatter(x[0, :], x[1, :])
    plt.axis("equal")
    plt.show()

rospy.Subscriber("cloud2d_projected", Projected, callback)
rospy.spin()





