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
    geometry_msgs::Point rpy;
    tf::Quaternion rpy_to_tf_quat(double roll, double pitch, double yaw);
  private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pub;
    geometry_msgs::Pose joy_pose;

     };

Frame_pub::Frame_pub()

{   sub = nh.subscribe("joy", 10, &Frame_pub::callback,this);
    pub = nh.advertise<geometry_msgs::PoseStamped>("/end_effector_pose", 1);
    pos.pose.position.x=0.154;
    pos.pose.position.y=0.133;
    pos.pose.position.z=0.784;
    pos.pose.orientation.x= -0.491;
    pos.pose.orientation.y= 0.601;
    pos.pose.orientation.z= -0.422;
    pos.pose.orientation.w= 0.469;
    // rpy.x=0;
    // rpy.y=90;
    // tf::Quaternion joy_quat2=rpy_to_tf_quat(-1.58,0,-1.1);
    // pos.pose.orientation.x= joy_quat2.getX();
    // pos.pose.orientation.y= joy_quat2.getY();
    // pos.pose.orientation.z= joy_quat2.getZ();
    // pos.pose.orientation.w= joy_quat2.getW();
    rpy.x=90;
    rpy.y=0;
    rpy.z=-80;
    // rpy.z=0;

    
}
tf::Quaternion Frame_pub::rpy_to_tf_quat(double roll, double pitch, double yaw){
  rpy.x=(roll)*(M_PI/180);
  rpy.y=(pitch)*(M_PI/180);
  rpy.z=(yaw)*(M_PI/180);
  ROS_INFO_STREAM("rpy-"<<rpy.x<<rpy.y<<rpy.z);
    return tf::createQuaternionFromRPY(rpy.x, rpy.y, rpy.z);
}
void Frame_pub::frame_pub() {
    
    pos.header.frame_id="world";
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
    joy_pose.position.z= int(data->axes[1])*0.0003;
    joy_pose.position.x= (-1.0*data->axes[0])*0.0003;
    joy_pose.position.y= (1.0*data->axes[7])*0.0003;
    //tf::Quaternion joy_quat=rpy_to_tf_quat(int(data->axes[3])*0.05,int(data->axes[4])*0.05,int(data->axes[6])*0.05);
      tf::Quaternion joy_quat=rpy_to_tf_quat(int(data->axes[3])*0.05,int(data->axes[4])*0.05,int(data->axes[6])*0.05);
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