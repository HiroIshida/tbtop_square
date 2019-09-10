#!/usr/bin/env python
import rospy 
import rospkg
import csv
import matplotlib.pyplot as plt
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point
from tbtop_square.msg import Projected
from math import *
import numpy as np
from scipy import stats
from scipy import optimize
import time

from detect_square import detect_rect
import cv2


class SquareDetector:
    def __init__(self, n_ave = 5):
        self.sub = rospy.Subscriber("cloud2d_projected", Projected, self.callback)
        self.pub = rospy.Publisher("s_est_pointcloud", Point, queue_size = 10)
        self.s_queue = []

    def callback(self, msg):
        x1 = np.array(msg.x_array.data)
        x2 = np.array(msg.y_array.data)
        x = np.vstack((x1, x2))
        s_est_ = detect_rect(x)
        print s_est_
        s_est = Point(x = s_est_[0], y = s_est_[1], z = s_est_[2])
        self.pub.publish(s_est)


if __name__=='__main__':
    rospy.init_node("detect_square", anonymous = True)
    sd = SquareDetector()
    rospy.spin()





