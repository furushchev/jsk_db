#!/usr/bin/env python


import rospy
from mongodb_store.message_store import MessageStoreProxy
from sensor_msgs.msg import Image
import message_filters as F

class ImageObjectAnnotator(object):
    def __init__(self):
        self.db_name = rospy.get_param("robot/database", "jsk_robot_lifelog")
        try:
            self.col_name = rospy.get_param("robot/name")
        except KeyError as e:
            rospy.logerr('please set param "robot/name" (e.g. pr1012)')
            exit(1)
        self.db = MessageStoreProxy(database=self.db_name, collection=self.col_name)
        rospy.loginfo("[{cls}] connected to {db}.{col}".format(cls=self.__cls__.__name__,
                                                               db=self.db_name,
                                                               col=self.col_name))
        self.img_sub = F.Subscriber("~image", Image)
        self.label_sub = F.Subscriber("~label", Image)
        self.sync = F.TimeSynchronizer([self.img_sub, self.label_sub], 20)
        self.sync.registerCallback(self.callback)
    def callback(self, img, lbl):
        pass
        

