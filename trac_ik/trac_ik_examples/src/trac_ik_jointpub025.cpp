#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/frames_io.hpp>
#include <stdio.h>
#include <iostream>

 #include <ros/ros.h>
 #include <sensor_msgs/JointState.h>
 #include <std_msgs/Header.h>
 #include <trajectory_msgs/JointTrajectory.h>

using namespace KDL;

sensor_msgs::JointState current_joint;
void chatterCallback(const sensor_msgs::JointState& joint)
{
  ROS_INFO_STREAM("joint_name"<<joint.name[0]);
  current_joint=joint;
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "vis_joint_publisher");
  ros::NodeHandle nh;
  ros::Publisher joint_pub = nh.advertise<trajectory_msgs::JointTrajectory>("arm_controller/command", 10);
  ros::Subscriber sub = nh.subscribe("joint_states", 10, chatterCallback);
  
  ros::Rate loop_rate(500);

trajectory_msgs::JointTrajectory tr0;
tr0.header.frame_id ="base_link";
tr0.joint_names.resize(6);
tr0.points.resize(1);

tr0.joint_names[0] = "shoulder_pan_joint";
tr0.joint_names[1] = "shoulder_lift_joint";
tr0.joint_names[2] = "elbow_joint";
tr0.joint_names[3] ="wrist_1_joint";
tr0.joint_names[4] ="wrist_2_joint";
tr0.joint_names[5] = "wrist_3_joint";

tr0.points[0].positions.resize(6);
tr0.points[0].positions[0]=0;
tr0.points[0].positions[1]=-1.8;
tr0.points[0].positions[2]=0.66;
tr0.points[0].positions[3]=1.14;
tr0.points[0].positions[4]=1.56;
tr0.points[0].positions[5]=-3.14;


  while (ros::ok())
  {

    tr0.header.stamp = ros::Time::now();
    tr0.points[0].time_from_start = ros::Duration(0.5);
    joint_pub.publish(tr0);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}