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
    def __init__(self, N, n):
        self.N = N
        self.n = n
        self.data = [np.zeros(self.n) for i in range(N)]

    def push(self, elem):
        tmp = self.data[1:self.N]
        tmp.append(elem)
        self.data = tmp

    def mean(self):
        s_est_arr = np.vstack(self.data)
        s_est = np.mean(s_est_arr, axis=0)
        #s_est_lst = [np.mean(np.array([s[i] for s in self.data])) for i in range(self.N)]
        return np.array(s_est)

class S1Queue:
    def __init__(self, N):
        self.N = N
        self.data = [0 for n in range(N)]
        self.scale = 4

    def push(self, angle):
        tmp = self.data[1:self.N]
        tmp.append(angle)
        self.data = tmp

    def mean(self):
        angles4 = np.array(self.data) * 4.0
        vecs = [np.array([cos(angle), sin(angle)]) for angle in angles4]
        vec_mean = np.mean(vecs, axis=0)
        angle = atan2(vec_mean[1], vec_mean[0])/4.0
        return angle

        


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
        self.x_queue = MyQueue(n_ave, 2)
        self.angle_queue = S1Queue(6)
        self.br = tf.TransformBroadcaster()

    def callback(self, msg):
        x1 = np.array(msg.x_array.data)
        x2 = np.array(msg.y_array.data)
        x = np.vstack((x1, x2))
        center, angle, area = extract_rect(x.T)
        isValid = lambda area: area > 0.006 and area < 0.009

        if isValid(area):
            #s = np.array([center[0], center[1], angle])
            self.x_queue.push(center)
            self.angle_queue.push(angle)
            x_mean = self.x_queue.mean()
            angle_mean = self.angle_queue.mean() + pi/2
            if angle_mean > pi/2:
                angle_mean = angle_mean - pi/2

            x_mean_pub= Point(x = x_mean[0], y = x_mean[1], z = angle_mean)
            self.pub.publish(x_mean_pub)

            rot = tf.transformations.quaternion_from_euler(0, 0.0, angle_mean)
            trans = [x_mean[0],x_mean[1], 0.723]
            self.br.sendTransform(trans, rot, rospy.Time.now(), "can", "base_link")

if __name__=='__main__':
    rospy.init_node("detect_square", anonymous = True)
    sd = SquareDetector()
    rospy.spin()





