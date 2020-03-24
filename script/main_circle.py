#!/usr/bin/env python
import rospy 
import rospkg
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from tbtop_square.msg import Projected
from math import *
import numpy as np
from tf2_msgs.msg import TFMessage
import tf

import cv2
from cv_bridge import CvBridge


class MyQueue:
    def __init__(self, N):
        self.N = N
        self.data = [np.zeros(2) for n in range(N)]

    def push(self, elem):
        tmp = self.data[1:self.N]
        tmp.append(elem)
        self.data = tmp

    def mean(self):
        s_est_lst = [np.mean(np.array([s[i] for s in self.data])) for i in range(2)]
        return np.array(s_est_lst)

class CircleDetector:
    def __init__(self, n_ave = 3):
        self.sub = rospy.Subscriber("/cloud2d_projected", Projected, self.callback)
        self.pub = rospy.Publisher("/object_pose", Point, queue_size = 1)
        self.s_queue = MyQueue(n_ave)
        self.br = tf.TransformBroadcaster()

    def callback(self, msg):
        print("tbtop: msg recieved")
        x1 = np.array(msg.x_array.data)
        x2 = np.array(msg.y_array.data)
        x = np.vstack((x1, x2))
        b_min = np.min(x, axis=1)
        b_max = np.max(x, axis=1)
        x_mean = 0.5 * (b_min + b_max)
        self.s_queue.push(x_mean)

    def publish(self):
        msg = Point()
        x_averaged = list(self.s_queue.mean())

        rot = tf.transformations.quaternion_from_euler(0, 0.0, pi/4)
        trans = [x_averaged[0], x_averaged[1], 0.723]
        self.br.sendTransform(trans, rot, rospy.Time.now(), "object", "base_link")

        print("center: " + str(x_averaged))
        msg.x, msg.y = x_averaged
        msg.z = pi/4
        self.pub.publish(msg)

if __name__=='__main__':
    rospy.init_node("detect_circle", anonymous = True)
    cd = CircleDetector()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        cd.publish()
        rate.sleep()

