#!/usr/bin/env python
import rospy 
import rospkg
import csv
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from tbtop_square.msg import Projected
from math import *
import numpy as np
from scipy import stats
from scipy import optimize
import scipy
import time
import utils

from detect_square import detect_rect
import cv2
from cv_bridge import CvBridge

import tf

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

def extract_rect(X):
    rect = utils.minimum_bounding_rectangle(X)
    center = np.mean(rect, axis=0)
    p0 = rect[0, :]
    p1 = rect[1, :]
    p2 = rect[2, :]
    vec1 = p1 - p0
    vec2 = p2 - p1
    area = np.linalg.norm(vec1) * np.linalg.norm(vec2)
    theta = atan2(vec1[1], vec1[0]) % (pi/2)
    return center, theta, area

class SquareDetector:
    def __init__(self, n_ave = 6):
        self.sub = rospy.Subscriber("/cloud2d_projected", Projected, self.callback)
        self.pub = rospy.Publisher("/square_pose", Point, queue_size = 1)
        self.pub_img = rospy.Publisher("/tbtop_debug_image", Image)
        self.s_queue = MyQueue(n_ave)
        self.br = tf.TransformBroadcaster()

    def callback(self, msg):
        x1 = np.array(msg.x_array.data)
        x2 = np.array(msg.y_array.data)
        x = np.vstack((x1, x2))
        center, angle, area = extract_rect(x.T)
        isValid = lambda area: area > 0.006 and area < 0.009

        if isValid(area):
            s = np.array([center[0], center[1], angle])
            self.s_queue.push(s)
            s_mean = self.s_queue.mean()

            rot = tf.transformations.quaternion_from_euler(0, 0.0, s_mean[2])
            trans = [s_mean[0], s_mean[1], 0.723]
            self.br.sendTransform(trans, rot, rospy.Time.now(), "can", "base_link")

if __name__=='__main__':
    rospy.init_node("detect_square", anonymous = True)
    sd = SquareDetector()
    rospy.spin()





