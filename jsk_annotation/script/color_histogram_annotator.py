#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>


import cv2
import rospy
from jsk_annotation import ImageObjectAnnotator
from sensor_msgs.msg import Image
from collections import defaultdict

(
    RED,
    YELLOW,
    GREEN,
    CYAN,
    BLUE,
    MAGENTA,
    WHITE,
    BLACK,
    GRAY,
    COUNT
) = range(10)

class ColorHistogramAnnotator(ImageObjectAnnotator):
    def __init__(self):
        super(ColorHistogramAnnotator, self).__init__(True)
        self.min_s = rospy.get_param("min_satuation", 60.0)
        self.min_v = rospy.get_param("min_value", 60.0)
        self.max_v_black = rospy.get_param("max_value_black", 60.0)
        self.min_v_white = rospy.get_param("min_value_white", 120.0)
        self.color_num = 6
        color_range = 256.0 / self.color_num
        self.hue_table = [i * color_range + color_range / 2.0 + 0.5 for i in range(self.color_num)]
    def process(self, rgb, label, header):
        hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV_FULL)
        hist = [0.0] * (self.color_num + 3)
        for y in hsv:
            for h, s, v in y:
                if s > self.min_s or v > self.min_v:
                    if h < self.hue_table[RED]:
                        hist[RED] += 1
                    elif h < self.hue_table[YELLOW]:
                        hist[YELLOW] += 1
                    elif h < self.hue_table[GREEN]:
                        hist[GREEN] += 1
                    elif h < self.hue_table[CYAN]:
                        hist[CYAN] += 1
                    elif h < self.hue_table[BLUE]:
                        hist[BLUE] += 1
                    elif h < self.hue_table[MAGENTA]:
                        hist[MAGENTA] += 1
                    else:
                        hist[RED] += 1
                elif v <= self.max_v_black:
                    hist[BLACK] += 1
                elif v > self.min_v_white:
                    hist[WHITE] += 1
                else:
                    hist[GRAY] += 1
        s = hsv.shape[0] * hsv.shape[1]
        hist = map(lambda x: 1.0 * x / s, hist)
        rospy.loginfo("label({label}): {hist}".format(label=label, hist=hist))
        return hist


if __name__ == '__main__':
    rospy.init_node("color_histogram_annotator")
    a = ColorHistogramAnnotator()
    rospy.spin()
