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

class force_convert{

  public:
    force_convert();
    void broadcast_dynamic_tf();
    string ft_sensor_name="/wrench";
  private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Subscriber current_force_sub;

    ros::Timer timer_;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    tf2_ros::TransformBroadcaster dynamic_br_;
    geometry_msgs::Point current_force;
    void current_force_callback(const geometry_msgs::WrenchStampedConstPtr& msg);

};

force_convert::force_convert():nh(), tfBuffer_(), tfListener_(tfBuffer_)
{
  ros::NodeHandle pnh("~");
  pnh.getParam("ft_sensor_topic", ft_sensor_name);
  current_force_sub = nh.subscribe(ft_sensor_name,10,&force_convert::current_force_callback,this);//subscribe ft_sensor
}

//sub current force val
void force_convert::current_force_callback(const geometry_msgs::WrenchStampedConstPtr& msg){
  current_force.x=msg->wrench.force.x;
  current_force.y=msg->wrench.force.y;
  current_force.z=msg->wrench.force.z;
  ROS_INFO_STREAM("current force = "<<"x="<<current_force.x<<",y="<<current_force.y<<",y="<<current_force.z);
}
void force_convert::broadcast_dynamic_tf(){
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "wrist_3_link";
    transformStamped.child_frame_id = "force_link";
    transformStamped.transform.translation.x = current_force.x ;
    transformStamped.transform.translation.y = current_force.y ;
    transformStamped.transform.translation.z = current_force.z;
    tf2::Quaternion q;
    q.setRPY(0, 0, 0.0);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    dynamic_br_.sendTransform(transformStamped);
}

int main( int argc, char** argv )
{

  ros::init(argc, argv, "force_tf_publisher");

  force_convert my1;

  ros::Rate loop_rate(500);//500Hz
  while (ros::ok())
  {
    my1.broadcast_dynamic_tf();
    ros::spinOnce();
    loop_rate.sleep();
  }

}