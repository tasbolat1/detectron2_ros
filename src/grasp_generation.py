#!/usr/bin/env python


# Objective
# Given mask on point clouds, generate top down grasp

import sys
import threading
import time

import rospy
import cv2 as cv
import numpy as np
from detectron2_ros.msg import Result
from sensor_msgs.msg import Image, RegionOfInterest
from cv_bridge import CvBridge, CvBridgeError
from sklearn.decomposition import PCA


objects = {
    'bottle': 0.1,
}

class GraspGenerationNode(object):
    def __init__(self):
        self._last_depth_msg = None
        self._depth_msg_lock = threading.Lock()

        self._last_seg_msg = None
        self._seg_msg_lock = threading.Lock()
        self.bridge = CvBridge()

        # subscribe to depth_to_aligned
        self.aligned_depth = rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.callback_aligned_depth, queue_size=1)
        self.segmentation_result = rospy.Subscriber('/detectron2_ros/result', Result, self.callback_seg_result, queue_size=1)

        self.all_objects_found = False


    def run(self):

        rate = rospy.Rate(100)

        # Part 1: searh for the objects
        if not self.all_objects_found:
            while not rospy.is_shutdown():
                if self._seg_msg_lock.acquire(False):
                    seg_results = self._last_seg_msg
                    self._last_seg_msg = None
                    self._seg_msg_lock.release()
                else:
                    rate.sleep()
                    continue

                if seg_results is not None:
                    mask = self.bridge.imgmsg_to_cv2(seg_results.masks[0], seg_results.masks[0].encoding)
                    
                    
                    
                    # draw the line
                
                rate.sleep()


        # get desired objects

        # while not rospy.is_shutdown():
        #     if self._depth_msg_lock.acquire(False):
        #         depth_msg = self._last_depth_msg
        #         self._last_depth_msg = None
        #         self._depth_msg_lock.release()
        #     else:
        #         rate.sleep()
        #         continue

        #     if depth_msg is not None:
         
        #         depth_image = self.bridge.imgmsg_to_cv2(depth_msg, depth_msg.encoding)
                
        #         print(depth_image.shape)

        #     rate.sleep()

    def smallest_axis(self, mask):
        mask_xy = np.argwhere(mask != 0)
        center = mask_xy.mean(axis=0)

        # run pca
        pca = PCA(n_components=2)
        pca.fit(mask_xy)
        # choose smallest component
        comp = pca.components_[1]
        var = pca.explained_variance_[1]
        x1 = comp[0] - center[0]
        y1 = comp[1] - center[1]

        x2 = comp[0] + center[0]
        y2 = comp[1] + center[1]

        return x1,y1, x2, y2

    def callback_seg_result(self, msg):
        # SEGMENTATION CALLBACK
        rospy.logdebug("Get segmentation result")
        if self._seg_msg_lock.acquire(False):
            self._last_seg_msg = msg
            self._header = msg.header
            self._seg_msg_lock.release()


    def callback_aligned_depth(self, msg):
        # DEPTH MAP CALLBACK
        rospy.logdebug("Get aligned depth map")
        if self._depth_msg_lock.acquire(False):
            self._last_depth_msg = msg
            self._header = msg.header
            self._depth_msg_lock.release()



def main(argv):
    rospy.init_node('grasp_generation')
    node = GraspGenerationNode()
    node.run()

if __name__ == '__main__':
    main(sys.argv)