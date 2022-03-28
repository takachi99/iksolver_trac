#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import pathlib
import numpy as np
import yaml

import rospy
import rospkg

import tf2_ros
import tf_conversions
import rosparam

from geometry_msgs.msg import *
from std_msgs.msg import *
from std_srvs.srv import *

# get absolute path of my_modules
current_dir = str(pathlib.Path(__file__).resolve().parent)
# appened module path directory
sys.path.append(current_dir)


class tf2_pub():

    def __init__(self):



        #################
        # load ros param
        #################
        try:
            ft_sensor_topic_n = rosparam.get_param("force_convert/ft_sensor_topic")
        except :
            ft_sensor_topic_n ='/wrench'

        #######################
        # subscribe ft sensor
        #######################
        rospy.Subscriber(ft_sensor_topic_n,WrenchStamped, self.current_ft_callback)
        ##################################
        # make filtered wrench publisher
        ###################################
        self.filtered_ftsensor =  rospy.Publisher("/LowPass_filtered_wrench", WrenchStamped, queue_size=1)
        ##################################
        # make ft sensor reset server
        ##################################
        rospy.Service("ft_sensor_reset",Trigger,self.ft_reset_trigger)

        self.current_force = WrenchStamped()
        self.offset_force  = WrenchStamped()


    def current_ft_callback(self,data):

        self.current_force= data


    def ft_reset_trigger(self,data):

        self.offset_force = self.current_force
        rospy.loginfo("offset ft sensor :")
        print(self.offset_force.wrench.force)

        return TriggerResponse(
            success=True,
            message="NOW ft_sensor offset "
        )

    def publish_tf(self,):

        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = 'wrist_3_link'
        t.child_frame_id = 'force_link'
        t.transform.translation.x = self.current_force.wrench.force.x-self.offset_force.wrench.force.x
        t.transform.translation.y = self.current_force.wrench.force.y-self.offset_force.wrench.force.y
        t.transform.translation.z = self.current_force.wrench.force.z-self.offset_force.wrench.force.z
        q = tf_conversions.transformations.quaternion_from_euler(0., 0., 0.)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        br.sendTransform(t)
        # rospy.loginfo(t)

        filtered_wrench = WrenchStamped()
        filtered_wrench.header.frame_id = "wrist_3_link"
        filtered_wrench.wrench.force.x=self.current_force.wrench.force.x-self.offset_force.wrench.force.x
        filtered_wrench.wrench.force.y=self.current_force.wrench.force.y-self.offset_force.wrench.force.y
        filtered_wrench.wrench.force.z=self.current_force.wrench.force.z-self.offset_force.wrench.force.z
        filtered_wrench.wrench.torque.x =0
        filtered_wrench.wrench.torque.y =0
        filtered_wrench.wrench.torque.z =0
        self.filtered_ftsensor.publish(filtered_wrench)

        rospy.loginfo_once('publish wrench frame')
        rospy.loginfo_once('publish wrench')

if __name__ == '__main__':

    rospy.init_node('model_origin_pub', anonymous=True)

    frame_pub = tf2_pub()

    r = rospy.Rate(1000)
    while not rospy.is_shutdown():
        frame_pub.publish_tf()
        r.sleep()

