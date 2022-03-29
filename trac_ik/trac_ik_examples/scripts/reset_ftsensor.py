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

import copy

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
            ft_sensor_topic_n = rosparam.get_param("reset_ftsensor/ft_sensor_topic")
        except :
            ft_sensor_topic_n ='/wrench_ft300s'

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


        rospack=rospkg.RosPack()
        self.package_path=rospack.get_path("trac_ik_examples")

        # self.current_force = WrenchStamped()
        self.current_force = Point()

        self.offset_force  = Point()


    def current_ft_callback(self,data):
        a =0.1
        # self.current_force.x=a*self.current_force.x+(1-a)*data.wrench.force.x;
        # self.current_force.y=a*self.current_force.y+(1-a)*data.wrench.force.y;
        # self.current_force.z=a*self.current_force.z+(1-a)*data.wrench.force.z;

        self.current_force.x=data.wrench.force.x;
        self.current_force.y=data.wrench.force.y;
        self.current_force.z=data.wrench.force.z;

        # print(self.current_force)
        # self.current_force= data

    def get_current_force(self):
        return self.current_force

    def ft_reset_trigger(self,data):

        offset_force = copy.copy(self.get_current_force())
        rospy.loginfo("offset ft sensor :")
        print(offset_force)

        import yaml
        obj = { 'force': {  'x': offset_force.x,
                                'y': offset_force.y,
                            'z': offset_force.z},}

        save_dir =self.package_path+"/config/"+"ftsensor_offset_"+".yaml"

        with open(save_dir, 'w') as file:
            yaml.dump(obj, file)

        with open(save_dir) as file:
            data = yaml.safe_load(file)

        self.offset_force.x = data["force"]['x']
        self.offset_force.y = data["force"]['y']
        self.offset_force.z = data["force"]['z']

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

        # t.transform.translation.x = self.current_force.x
        # t.transform.translation.y = self.current_force.y
        # t.transform.translation.z = self.current_force.z

        t.transform.translation.x = self.current_force.x-self.offset_force.x
        t.transform.translation.y = self.current_force.y-self.offset_force.y
        t.transform.translation.z = self.current_force.z-self.offset_force.z

        q = tf_conversions.transformations.quaternion_from_euler(0., 0., 0.)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        br.sendTransform(t)
        # rospy.loginfo(t)

        filtered_wrench = WrenchStamped()
        filtered_wrench.header.frame_id = "wrist_3_link"

        # filtered_wrench.wrench.force.x=self.current_force.x
        # filtered_wrench.wrench.force.y=self.current_force.y
        # filtered_wrench.wrench.force.z=self.current_force.z

        filtered_wrench.wrench.force.x=self.current_force.x-self.offset_force.x
        filtered_wrench.wrench.force.y=self.current_force.y-self.offset_force.y
        filtered_wrench.wrench.force.z=self.current_force.z-self.offset_force.z

        filtered_wrench.wrench.torque.x =0
        filtered_wrench.wrench.torque.y =0
        filtered_wrench.wrench.torque.z =0
        self.filtered_ftsensor.publish(filtered_wrench)

        rospy.loginfo_once('publish wrench frame')
        rospy.loginfo_once('publish wrench')

        # rospy.loginfo(filtered_wrench)
if __name__ == '__main__':

    rospy.init_node('model_origin_pub', anonymous=True)

    frame_pub = tf2_pub()

    r = rospy.Rate(500)
    while not rospy.is_shutdown():
        frame_pub.publish_tf()
        r.sleep()

