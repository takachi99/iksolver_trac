#include <stdio.h>
#include <iostream>

#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float32MultiArray.h>

#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std;

class external_force_frame_pub{

  public:
    external_force_frame_pub();
  string ft_sensor_name="test";

    void frame_pub();

  private:
    ros::NodeHandle nh;
    ros::Timer timer_;
    ros::Publisher target_frame_pub;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    tf2_ros::TransformBroadcaster dynamic_br_;
    geometry_msgs::Point current_force;
    geometry_msgs::Pose current_pose;
    geometry_msgs::PoseStamped send_frame;

};

external_force_frame_pub::external_force_frame_pub():nh(), tfBuffer_(), tfListener_(tfBuffer_)
{
  ros::NodeHandle pnh("~");
  pnh.getParam("ft_sensor_topic", ft_sensor_name);
  target_frame_pub = nh.advertise<geometry_msgs::PoseStamped>("/end_effector_pose", 1); //pub frame to ik solver node

  //get end effector frame
  timer_ = nh.createTimer(ros::Duration(0.0001), [&](const ros::TimerEvent& e) {
    geometry_msgs::TransformStamped transformStamped;
    geometry_msgs::TransformStamped force_transform;
    try{
      transformStamped = tfBuffer_.lookupTransform("base_link", "tool0", ros::Time(0));
      force_transform = tfBuffer_.lookupTransform("base_link", "force_link", ros::Time(0));
      }
    catch (tf2::TransformException& ex){
      ROS_WARN("error->%s", ex.what());
      return;
    }

    //copy to geometry_msgs::PoseStamped send_frame
    send_frame.header.frame_id=transformStamped.header.frame_id;
    current_pose.position.x=transformStamped.transform.translation.x;
    current_pose.position.y=transformStamped.transform.translation.y;
    current_pose.position.z=transformStamped.transform.translation.z;
    current_pose.orientation.x=transformStamped.transform.rotation.x;
    current_pose.orientation.y=transformStamped.transform.rotation.y;
    current_pose.orientation.z=transformStamped.transform.rotation.z;
    current_pose.orientation.w=transformStamped.transform.rotation.w;
    ROS_INFO_STREAM("current_pose x= "<<current_pose);
    current_force.x=force_transform.transform.translation.x;
    current_force.y=force_transform.transform.translation.y;
    current_force.z=force_transform.transform.translation.z;
    });

}

void external_force_frame_pub::frame_pub(){
  send_frame.pose.position.x=current_pose.position.x+(current_force.x*0.0008);
  send_frame.pose.position.y=current_pose.position.y+(current_force.y*0.0008);
  send_frame.pose.position.z=current_pose.position.z+(current_force.z*0.0008);
  send_frame.pose.orientation.x=current_pose.orientation.x;
  send_frame.pose.orientation.y= current_pose.orientation.y;
  send_frame.pose.orientation.z= current_pose.orientation.z;
  send_frame.pose.orientation.w= current_pose.orientation.w;
  target_frame_pub.publish(send_frame);
  ROS_INFO_STREAM("pub_frame"<<"x="<<send_frame.pose.position.x<<" y= "<<send_frame.pose.position.y<<" z= "<<send_frame.pose.position.z);
  }

int main( int argc, char** argv )
{

  ros::init(argc, argv, "external_force_frame_pub");

  external_force_frame_pub my1;

  ros::Rate loop_rate(800);//500Hz
  while (ros::ok())
  {
    my1.frame_pub();
    ros::spinOnce();
    loop_rate.sleep();
  }

}