#!/usr/bin/env python

import cv2
import numpy as np
import rospy
from mongodb_store.message_store import MessageStoreProxy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import message_filters as F
from jsk_annotation.msg import ObjectAnnotation

class ImageObjectAnnotator(object):
    def __init__(self, crop_image=True):
        try:
            self.annotator_type = rospy.get_param("~annotator_type")
        except KeyError as e:
            rospy.logerr("please set annotator_type")
            exit(1)
        self.db_name = rospy.get_param("robot/database", "jsk_robot_lifelog")
        try:
            self.col_name = rospy.get_param("robot/name")
        except KeyError as e:
            rospy.logerr('please set param "robot/name" (e.g. pr1012)')
            exit(1)
        self.img_queue_size = rospy.get_param("image_queue_size", 50)
        self.sync_timeout = rospy.get_param("sync_timeout", 0.1)
        self.crop_image_enable = crop_image
        self.db = MessageStoreProxy(database=self.db_name, collection=self.col_name)
        rospy.loginfo("[{cls}] connected to {db}.{col}".format(cls=type(self).__name__,
                                                               db=self.db_name,
                                                               col=self.col_name))
        self.annotation_pub = rospy.Publisher("~annotation", ObjectAnnotation, queue_size=10)
        self.bridge = CvBridge()
        self.img_sub = F.Subscriber("~image", Image)
        self.label_sub = F.Subscriber("~label", Image)
        self.sync = F.ApproximateTimeSynchronizer([self.img_sub, self.label_sub],
                                                  self.img_queue_size, self.sync_timeout)
        self.sync.registerCallback(self.callback)
    def callback(self, imgmsg, lblmsg):
        rospy.loginfo("callback")
        header = imgmsg.header
        try:
            img = self.bridge.imgmsg_to_cv2(imgmsg, "bgr8")
            lbl = self.bridge.imgmsg_to_cv2(lblmsg)
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        rospy.logdebug("img.size: {img}, label.size: {lbl}".format(img=img.shape, lbl=lbl.shape))
        for label in np.unique(lbl):
            if label == 0:
                continue
            masked = img.copy()
            masked[(lbl == label) == False] = 0
            if self.crop_image_enable:
                masked = self.crop_image(masked)
            annotation = self.process(masked, label-1, header)
            msg = ObjectAnnotation()
            msg.header = header
            msg.name = label-1
            msg.type = self.annotator_type
            if type(annotation) == str:
                msg.string_value = annotation
            elif type(annotation) == list and len(annotation) > 0 and type(annotation[0]) == float:
                msg.float_value = annotation
            elif annotation is None:
                continue
            else:
                raise ValueError("annotation type must be str / float")
            self.annotation_pub.publish(msg)
    def crop_image(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        contours = cv2.findContours(gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        max_contour = None
        max_area = 0
        for c in contours[0]:
            area = cv2.contourArea(c)
            if max_area < area:
                max_area = area
                max_contour = c
        brect = cv2.boundingRect(max_contour)
        return img[brect[1]:brect[1]+brect[3],brect[0]:brect[0]+brect[2]]
    def process(self, img, label):
        raise NotImplementedError("[{cls}] method process is not implemented".format(cls=type(self).__name__))
        # return annotation

if __name__ == '__main__':
    rospy.init_node("object_annotator")
    a = ImageObjectAnnotator()
    rospy.spin()
