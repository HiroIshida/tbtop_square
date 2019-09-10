#!/usr/bin/env python
import matplotlib.pyplot as plt
from math import *
import numpy as np
import time
from utils import *
import cv2


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
