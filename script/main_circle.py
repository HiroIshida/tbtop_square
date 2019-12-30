#!/usr/bin/env python
import rospy 
import rospkg
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from tbtop_square.msg import Projected
from math import *
import numpy as np

from detect_square import detect_rect
import cv2
from cv_bridge import CvBridge

class MyQueue:
    def __init__(self, N):
        self.N = N
        self.data = [np.zeros(3) for n in range(N)]

    def push(self, elem):
        tmp = self.data[1:self.N]
        tmp.append(elem)
        self.data = tmp

    def mean(self):
        s_est_lst = [np.mean(np.array([s[i] for s in self.data])) for i in range(3)]
        return np.array(s_est_lst)

class CircleDetector:
    def __init__(self, n_ave = 10):
        self.sub = rospy.Subscriber("/cloud2d_projected", Projected, self.callback)
        self.pub = rospy.Publisher("/circle_center", Point, queue_size = 1)

    def callback(self, msg):
        print("hoge")
        x1 = np.array(msg.x_array.data)
        x2 = np.array(msg.y_array.data)
        x = np.vstack((x1, x2))
        x_mean = np.mean(x, axis = 1)
        x_mean_lst = list(x_mean)
        print(x_mean_lst)

        msg = Point()
        msg.x, msg.y = x_mean_lst
        self.pub.publish(msg)

if __name__=='__main__':
    rospy.init_node("detect_circle", anonymous = True)
    cd = CircleDetector()
    rospy.spin()




