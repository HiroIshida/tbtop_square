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
import time

from utils import *
import cv2

img = cv2.imread("../model/image_raw_screenshot_3.png")
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

jdict = json_read()
x = jdict["x"]

def pts2img(x, N, margin = 1):
    x0 = x[0]
    x1 = x[1]

    def indexise(arr_, N):
        arr_ = np.array(arr_)
        bmin_ = min(arr_)
        bmax_ = max(arr_)
        w = bmax_ - bmin_
        bmin = bmin_ - w * margin
        bmax = bmax_ + w * margin

        grid_size = (bmax - bmin)/N
        arr = arr_ - bmin
        idxes = (arr - arr % grid_size)/grid_size
        return idxes.astype("int"), bmin, bmax

    idxes0_, bmin0, bmax0 = indexise(x0, N)
    idxes1_, bmin1, bmax1 = indexise(x1, N)
    bmin = np.array([bmin0, bmin1])
    bmax = np.array([bmax0, bmax1])
    gsize = (np.array(bmax) - np.array(bmin))/N
    idxes_list = [(idxes0_[i], idxes1_[i]) for i in range(idxes0_.size)]
    idxes_set = set(idxes_list)

    img = np.zeros((N, N), dtype = "uint8")
    for pair in idxes_set:
        img[pair[0], pair[1]] = 255

    return img, bmin, gsize

def detect_rect(x, debug = False):
    debug = False
    img, bmin, dx = pts2img(x, 100)
    img = cv2.blur(img, (10, 10))
    _, img_t = cv2.threshold(img, 100, 255, 0)

    _, contours, _ = cv2.findContours(img_t, 1, 2)
    cnt = contours[0]
    rect = cv2.minAreaRect(cnt)
    box = cv2.boxPoints(rect)
    box = np.int0(box)

    if debug:
        _, img_debug = cv2.threshold(img, 100, 100, 0)
        for pair in box:
            img_debug[pair[0], pair[1]] = 255
        cv2.imshow("image", img_debug)
        cv2.waitKey(2)
        time.sleep(3)
        cv2.destroyWindow("image")

    points = []
    for pair in box:
        points.append(bmin + pair * dx)
    x_mean = np.mean(np.array([p[0] for p in points]))
    y_mean = np.mean(np.array([p[1] for p in points]))

    vecs = []
    for pair_idx in [[0, 1], [1, 2], [2, 3], [3, 0]]:
        vec_ = points[pair_idx[1]] - points[pair_idx[0]]
        vec = vec_/np.linalg.norm(vec_)
        vecs.append(vec)

    angle = acos(vecs[0][0]) % (pi/2)
    s_est = [x_mean, y_mean, angle]
    return s_est






"""
cv2.imshow('imge', img_t)
cv2.waitKey(0)
"""




"""
cv2.destroyWindow("image")
"""













"""

def csvwrite(x):
    with open('./tmp.csv', 'w') as f:
        writer = csv.writer(f)
        N = x.shape[1]
        for i in range(N):
            writer.writerow([x[0, i], x[1, i]])

def Rot(a):
    m = np.matrix([[cos(a), -sin(a)], [sin(a), cos(a)]])
    return m

def evaluate(mu, angle, x_data, w):
    N = x_data.shape[1]
    x_err_ = x_data - np.tile(mu, [N, 1]).T
    x_err = Rot(-angle).dot(x_err_)
    tmp = np.where(abs(x_err) < w * 0.5, True, False)
    boolean =  np.logical_and(tmp[0, :], tmp[1, :]) 
    return sum(boolean)*1.0/N


def compute_minimum_w(mu, angle, x_data):
    w_left = 0.03
    w_right = 0.20
    f = lambda w: evaluate(mu, angle, x_data, w) - 1.0

    for i in range(20):
        fl = f(w_left)
        fr = f(w_right)
        if fl == 0.0:
            tmp = w_left
            w_left = w_left - (w_right - w_left)
            w_right = tmp
        elif fl < fr:
            w_left = 0.5 * (w_left + w_right)
        else:
            w_right = 0.5 * (w_left + w_right)
    return 0.5 * (w_left + w_right)

def compute_optimal_angle(x_data):
    mu = np.mean(x_data, axis = 1)
    for rad in [0 + 3*i
    fn = lambda angle: compute_minimum_w(mu, angle, x_data)
    ret = optimize.minimize(fn, 0, method = 'Nelder-Mead')
    return ret
    """

"""
rospy.init_node("detect_square", anonymous = True)
pub = rospy.Publisher("s_est_pointcloud", Point, queue_size = 10)

global angle_mean
angle_mean = None

global counter
counter = 1



global x
def callback(msg):
    x1 = np.array(msg.x_array.data)
    x2 = np.array(msg.y_array.data)
    global x
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
    std_1 = evaluate(mu, angle_est, x_data, lambda x: np.max(abs(x)))
    std_2 = evaluate(mu, angle_est + pi/4 , x_data, lambda x: np.max(abs(x)))
    angle_est = (angle_est if std_1 < std_2 else angle_est + pi/4)

    global angle_mean
    global counter
    angle_est = angle_est % (pi * 0.5)
    if angle_mean is None:
        angle_mean = angle_est
    else:
        angle_mean = angle_est /(counter +1) + (counter /(counter + 1)) * angle_mean
    counter += 1

    s_est = np.array([mu[0], mu[1], angle_mean])

    return s_est

def show():
    global x
    plt.scatter(x[0, :], x[1, :])
    plt.axis("equal")
    plt.show()

rospy.Subscriber("cloud2d_projected", Projected, callback)
rospy.spin()
"""





