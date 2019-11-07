#!/usr/bin/env python
from threading import Thread, RLock
from collections import deque
from copy import deepcopy
import math

import rospy
from tf2_ros import TransformBroadcaster
from tf.transformations import quaternion_from_euler, euler_from_quaternion, compose_matrix, inverse_matrix, translation_from_matrix, quaternion_from_matrix

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Point, TransformStamped, Quaternion, PoseArray

class OdomLocalizer(object):
    def __init__(self):
        # Odometry and pose
        self.odom = Odometry
        self.pose = PoseStamped(); self.pose.pose.orientation.w = 1.0
        
        # Pose array which will contain all the estimated poses
        self.pose_array = PoseArray()
        self.pose_array_maximum_size = None
        
        # Used in automatic subscribing and publishing
        self.pose_publisher = None
        self.pose_array_publisher = None

        # Odometry subscriber
        self.odom_subscriber = None

    def set_odom(self, odom):
        self.odom = odom
        self.update_pose()

    def get_pose(self):
        return self.pose

    def update_pose(self):
        # Update the frame id and time stamp
        self.pose.header.frame_id = self.odom.header.frame_id
        self.pose.header.stamp = rospy.Time.now()
        
        # Update the pose
        self.pose.pose = self.odom.pose.pose

        # Publish trajectory if enabled
        if self.pose_array_publisher != None:
            self.pose_array.header = self.pose.header
            self.pose_array.poses.append(deepcopy(self.pose.pose))

            # If a maximum size has been set, keep the pose array at maximum size
            if(self.pose_array_maximum_size != None):
                if(len(self.pose_array.poses) > self.pose_array_maximum_size):
                    self.pose_array.poses.pop(0)

            self.pose_array_publisher.publish(self.pose_array)

        # Publish the pose if enabled
        if(self.pose_publisher != None):
            self.pose_publisher.publish(self.pose)

    def enable_odom_subscribing(self, topic):
        self.odom_subscriber = rospy.Subscriber(topic, Odometry, callback=self.set_odom)

    def enable_pose_publishing(self, topic):
        # Instanciate publisher
        self.pose_publisher = rospy.Publisher(topic, PoseStamped, queue_size=1)
    
    def enable_pose_array_publishing(self, topic, maximum_size=None):
        # Set the maximum size, if any
        self.pose_array_maximum_size = maximum_size
        # Instanciate publisher
        self.pose_array_publisher = rospy.Publisher(topic, PoseArray, queue_size=1)
