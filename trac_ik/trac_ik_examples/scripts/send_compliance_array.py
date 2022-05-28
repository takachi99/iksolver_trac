#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from tqdm import tqdm
import numpy as np
import copy
import time
import os



import rospy
import rospkg

import tf
import tf2_ros
import tf2_geometry_msgs  # **Do not use geometry_msgs. Use this instead for PoseStamped


from geometry_msgs.msg import *
from std_msgs.msg import *
from std_srvs.srv import *




class compliance_pub():

    def __init__(self):
        # frame publisher
        self.target_pose_pub =  rospy.Publisher("/array",Float32MultiArray, queue_size=1)

    def euler_to_quaternion(self,euler):
        """
        Convert Euler Angles to Quaternion
        euler: geometry_msgs/Vector3
        quaternion: geometry_msgs/Quaternion
        """
        q = tf.transformations.quaternion_from_euler(euler.x, euler.y, euler.z)
        return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

    def quaternion_to_euler(self,quaternion):
        """
        Convert Quaternion to Euler Angles
        quaternion: geometry_msgs/Quaternion
        euler: geometry_msgs/Vector3
        """
        e = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
        return Vector3(x=e[0], y=e[1], z=e[2])



    def transform_pose(self,input_pose, from_frame, to_frame):
        """[summary]
        transform between two frame

        Args:
            input_pose ([type]): geometry.msgs.Pose
            from_frame ([type]): header.frame_id
            to_frame ([type]): header.frame_id

        Returns:
            [type]: [description]
        """
        pose_stamped = tf2_geometry_msgs.PoseStamped()
        pose_stamped.pose = input_pose
        pose_stamped.header.frame_id = from_frame
        pose_stamped.header.stamp = rospy.Time.now()
        try:
            # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
            output_pose_stamped = self.tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(0.001))
            # recommend tf lookup trans form
            # output_pose_stamped = self.tf_buffer.transform(pose_stamped, to_frame, rospy.Time(0))
            return output_pose_stamped

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise



    def publish(self):



        target_config = [0.0]*12

        target_config[0] = -0.116
        target_config[1] =0.493
        target_config[2] =0.5

        quat =self.euler_to_quaternion(Vector3(np.deg2rad(-90),0,0))

        target_config[3] = quat.x
        target_config[4] = quat.y
        target_config[5] = quat.z
        target_config[6] = quat.w
        target_config[7] = 0.0
        target_config[8] = -10.0
        target_config[9] = 0.0
        target_config[10]= 1.0
        target_config[11]= 1.0

        array_forPublish = Float32MultiArray(data=target_config)
        self.target_pose_pub.publish(array_forPublish)
        rate.sleep()



if __name__ == '__main__':

    rospy.init_node('simple_compliance_node_pub', anonymous=True)
    ctrl = compliance_pub()

    rate = rospy.Rate(500)
    while not rospy.is_shutdown():
        ctrl.publish()
        rate.sleep()


