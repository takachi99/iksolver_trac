#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/frames_io.hpp>

#include <trac_ik/trac_ik.hpp>

#include <stdio.h>
#include <iostream>
#include <math.h>
 #include <ros/ros.h>
 #include <sensor_msgs/JointState.h>
 #include <std_msgs/Header.h>
 #include <trajectory_msgs/JointTrajectory.h>
 #include <geometry_msgs/PoseStamped.h>
 #include <sensor_msgs/Joy.h>
 #include <geometry_msgs/Pose.h>

 #include <tf/transform_broadcaster.h>


class Frame_pub{

    public:
    Frame_pub();
    void frame_pub();
    void callback(const sensor_msgs::Joy::ConstPtr& data);
    geometry_msgs::PoseStamped pos;
    
    tf::Quaternion rpy_to_tf_quat(double roll, double pitch, double yaw);
    geometry_msgs::Point orientation_count(double x, double y, double z);
  private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pub;
    geometry_msgs::Pose joy_pose;
    geometry_msgs::Point rpy_diff;
     };

Frame_pub::Frame_pub()

{   sub = nh.subscribe("joy", 10, &Frame_pub::callback,this);
    pub = nh.advertise<geometry_msgs::PoseStamped>("/end_effector_pose", 1);
    pos.pose.position.x=-0.129;
    pos.pose.position.y=0.392;
    pos.pose.position.z=0.524;
    joy_pose.orientation.x= -0.707;
    joy_pose.orientation.y= 0.0;
    joy_pose.orientation.z= 0.0;
    joy_pose.orientation.w= 0.707;
    // rpy.x=0;
    // rpy.y=90;
    // tf::Quaternion joy_quat2=rpy_to_tf_quat(-1.58,0,-1.1);
    // pos.pose.orientation.x= joy_quat2.getX();
    // pos.pose.orientation.y= joy_quat2.getY();
    // pos.pose.orientation.z= joy_quat2.getZ();
    // pos.pose.orientation.w= joy_quat2.getW();

}
tf::Quaternion Frame_pub::rpy_to_tf_quat(double roll, double pitch, double yaw){
  geometry_msgs::Point rpy;
  rpy.x=(roll)*(M_PI/180);
  rpy.y=(pitch)*(M_PI/180);
  rpy.z=(yaw)*(M_PI/180);
  ROS_INFO_STREAM("rpy-"<<rpy.x<<rpy.y<<rpy.z);
    return tf::createQuaternionFromRPY(rpy.x, rpy.y, rpy.z);
}
geometry_msgs::Point Frame_pub::orientation_count(double x, double y, double z){
  rpy_diff.x=rpy_diff.x+x;
  rpy_diff.y=rpy_diff.y+y;
  rpy_diff.z=rpy_diff.z+z;
  ROS_INFO_STREAM("rpy_diff"<<rpy_diff.x<<rpy_diff.y<<rpy_diff.z);
  return rpy_diff;
}

void Frame_pub::frame_pub() {
    
    pos.header.frame_id="base_link";
    pos.pose.position.x=pos.pose.position.x+joy_pose.position.x;
    pos.pose.position.y=pos.pose.position.y+joy_pose.position.y;
    pos.pose.position.z = pos.pose.position.z+joy_pose.position.z;
    pos.pose.orientation.x= joy_pose.orientation.x;
    pos.pose.orientation.y= joy_pose.orientation.y;
    pos.pose.orientation.z= joy_pose.orientation.z;
    pos.pose.orientation.w= joy_pose.orientation.w;
    // pos.pose.orientation.x= -0.491;
    // pos.pose.orientation.y= 0.601;
    // pos.pose.orientation.z= -0.422;
    // pos.pose.orientation.w= 0.469;

    pub.publish(pos);
  }

void Frame_pub::callback(const sensor_msgs::Joy::ConstPtr& data){
    joy_pose.position.z= (data->axes[1])*0.0003;
    joy_pose.position.x= (data->axes[7])*0.0003;
    joy_pose.position.y= (data->axes[0])*0.0003;
    //tf::Quaternion joy_quat=rpy_to_tf_quat(int(data->axes[3])*0.05,int(data->axes[4])*0.05,int(data->axes[6])*0.05);
    geometry_msgs::Point diff=orientation_count((data->axes[4])*0.5,(data->axes[6])*0.5,(data->axes[3])*0.5);
      tf::Quaternion joy_quat=rpy_to_tf_quat(-90+diff.x,0+diff.y,0+diff.z);
    joy_pose.orientation.x=joy_quat.getX();
    joy_pose.orientation.y=joy_quat.getY();
    joy_pose.orientation.z=joy_quat.getZ();
    joy_pose.orientation.w=joy_quat.getW();
}

int main( int argc, char** argv )
{

ros::init(argc, argv, "my_frame_publisher");

Frame_pub my1;

  ros::Rate loop_rate(500);
  while (ros::ok())
  { 
      my1.frame_pub();
    ros::spinOnce();
    loop_rate.sleep();
  }

}