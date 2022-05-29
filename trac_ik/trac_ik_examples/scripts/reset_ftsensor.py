#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import pathlib
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
            # ft_sensor_topic_n ='/wrench'
            

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

        self.current_force = WrenchStamped()
        self.offset_force  = WrenchStamped()


        # self.current_force = Point()
        # self.offset_force  = Point()


    def current_ft_callback(self,data):

        # Low pass
        a =0.8




        self.current_force.wrench.force.x=a*self.current_force.wrench.force.x+(1-a)*data.wrench.force.x;
        self.current_force.wrench.force.y=a*self.current_force.wrench.force.y+(1-a)*data.wrench.force.y;
        self.current_force.wrench.force.z=a*self.current_force.wrench.force.z+(1-a)*data.wrench.force.z;

        self.current_force.wrench.torque.x=a*self.current_force.wrench.torque.x+(1-a)*data.wrench.torque.x
        self.current_force.wrench.torque.y=a*self.current_force.wrench.torque.y+(1-a)*data.wrench.torque.y
        self.current_force.wrench.torque.z=a*self.current_force.wrench.torque.z+(1-a)*data.wrench.torque.z



        # self.current_force.wrench.force.x=data.wrench.force.x
        # self.current_force.wrench.force.y=data.wrench.force.y
        # self.current_force.wrench.force.z=data.wrench.force.z

        # self.current_force.wrench.torque.x=data.wrench.torque.x
        # self.current_force.wrench.torque.y=data.wrench.torque.y
        # self.current_force.wrench.torque.z=data.wrench.torque.z


        # print(self.current_force)

    def get_current_force(self):
        return self.current_force

    def ft_reset_trigger(self,data):

        offset_force = copy.copy(self.get_current_force())
        rospy.loginfo("Offset FT sensor val :")
        print(offset_force)

        obj = { 'force': {  'x': offset_force.wrench.force.x,
                            'y': offset_force.wrench.force.y,
                            'z': offset_force.wrench.force.z},
               'torque': {  'x': offset_force.wrench.torque.x,
                            'y': offset_force.wrench.torque.y,
                            'z': offset_force.wrench.torque.z},}

        save_dir =self.package_path+"/config/"+"ft_sensor_offset"+".yaml"

        with open(save_dir, 'w') as file:
            yaml.dump(obj, file)

        with open(save_dir) as file:
            data = yaml.safe_load(file)

        self.offset_force.wrench.force.x  = data["force"]['x']
        self.offset_force.wrench.force.y  = data["force"]['y']
        self.offset_force.wrench.force.z  = data["force"]['z']
        self.offset_force.wrench.torque.x = data["torque"]['x']
        self.offset_force.wrench.torque.y = data["torque"]['y']
        self.offset_force.wrench.torque.z = data["torque"]['z']


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

        filtered_wrench.wrench.torque.x =self.current_force.wrench.torque.x-self.offset_force.wrench.torque.x
        filtered_wrench.wrench.torque.y =self.current_force.wrench.torque.y-self.offset_force.wrench.torque.y
        filtered_wrench.wrench.torque.z =self.current_force.wrench.torque.z-self.offset_force.wrench.torque.z

        self.filtered_ftsensor.publish(filtered_wrench)

        rospy.loginfo_once('start publish wrench frame..')
        rospy.loginfo_once('start publish wrench')


if __name__ == '__main__':

    rospy.init_node('reset_ft_sensor', anonymous=True)

    frame_pub = tf2_pub()

    r = rospy.Rate(500)
    while not rospy.is_shutdown():
        frame_pub.publish_tf()
        r.sleep()

