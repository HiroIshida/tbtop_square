#!/usr/bin/env python
import matplotlib.pyplot as plt
from math import *
import numpy as np
import time
from utils import *
import cv2

def pts2img(x, N, margin = 0.3):
    """
    not only convert points into image, check validity interms of ratio of pixels of objects 
    to the whole pixels
    """
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
    idxes_set = [x for x in set(idxes_list) if idxes_list.count(x) > 1]
    rate = len(idxes_set)*1.0/(N**2)
    print rate
    isInvalid = rate < 0.05

    img = np.zeros((N, N), dtype = "uint8")
    for pair in idxes_set:
        img[pair[0], pair[1]] = 255

    return img, bmin, gsize, isInvalid

def check_validity(img, box):
    n_filled_pixel = img[img > 0.0].size

    def guess_pixels_in_box(box):
        x_box, y_box = [[e[i] for e in box] for i in range(2)]
        [[x_min, x_max], [y_min, y_max]] = [[fn(e) for fn in [min, max]] for e in [x_box, y_box]]
        return (x_max - x_min) * (y_max - y_min)

    isValid = (guess_pixels_in_box(box)/n_filled_pixel > 0.8)
    return isValid



def detect_rect(x, debug = False):
    debug = False
    img, bmin, dx, isInvalid_pixels = pts2img(x, 50)
    img = cv2.blur(img, (7, 7))
    _, img_t = cv2.threshold(img, 1, 255, 0)

    _, contours, _ = cv2.findContours(img_t, 1, 2)
    cnt = contours[0]
    rect = cv2.minAreaRect(cnt)
    box = cv2.boxPoints(rect)
    box = np.int0(box)

    _, img_debug = cv2.threshold(img_t, 100, 50, 0)

    for pair in box:
        img_debug[pair[1], pair[0]] = 255

    for pair_ in cnt:
        pair = pair_[0]
        img_debug[pair[1], pair[0]] = 130

    if debug:
        cv2.imshow("image", img_debug)
        cv2.waitKey(2)
        time.sleep(3)
        cv2.destroyWindow("image")

    points = []
    for pair in box:
        points.append(bmin + pair * dx)
    #x_mean = np.mean(np.array([p[0] for p in points]))
    #y_mean = np.mean(np.array([p[1] for p in points]))

    vecs = []
    lens = []
    for pair_idx in [[0, 1], [1, 2], [2, 3], [3, 0]]:
        vec_ = points[pair_idx[1]] - points[pair_idx[0]]
        leng = np.linalg.norm(vec_)
        vec = vec_/leng
        vecs.append(vec)
        lens.append(leng)

    size = lens[0] * lens[1]
    angle = acos(vecs[0][0]) % (pi/2)
    x_mean = np.mean(x[0])
    y_mean = np.mean(x[1])
    s_est = [x_mean, y_mean, angle]

    isInvalid = not check_validity(img_t, box)
    return s_est, img_debug, isInvalid

