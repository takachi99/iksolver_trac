#include <stdio.h>
#include <iostream>

 #include <ros/ros.h>

 #include <std_msgs/Header.h>
 #include <trajectory_msgs/JointTrajectory.h>
 #include <geometry_msgs/PoseStamped.h>
 #include <sensor_msgs/Joy.h>
 #include <geometry_msgs/Pose.h>

 #include <tf/transform_broadcaster.h>
#include <std_msgs/Float32MultiArray.h>

class Frame_pub{

    public:
    Frame_pub();
    void frame_pub();
    void callback(const sensor_msgs::Joy::ConstPtr& data);
    geometry_msgs::Point rpy;
    std_msgs::Float32MultiArray array;
    tf::Quaternion rpy_to_tf_quat(double roll, double pitch, double yaw);
    geometry_msgs::Point orientation_count(double x, double y, double z);
  private:
  private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pub;
    ros::Publisher pub1;

    geometry_msgs::Pose joy_pose;
    geometry_msgs::Point rpy_diff;
    geometry_msgs::PoseStamped target_pose;

     };

Frame_pub::Frame_pub()

{   sub = nh.subscribe("joy", 10, &Frame_pub::callback,this);
    pub = nh.advertise<std_msgs::Float32MultiArray>("/array", 1);
    pub1 = nh.advertise<geometry_msgs::PoseStamped>("target_frame",1);
    array.data.resize(10);
    array.data[0] = -0.129;//x
    array.data[1] = 0.392;//y
    array.data[2] = 0.524;//z

    array.data[3] = -0.707;
    array.data[4] = 0.0;
    array.data[5] = 0.0;
    array.data[6] = 0.707;

    joy_pose.orientation.x= -0.707;
    joy_pose.orientation.y= 0.0;
    joy_pose.orientation.z= 0.0;
    joy_pose.orientation.w= 0.707;
    

    array.data[7] = 0.0;
    array.data[8] = 0.0;
    array.data[9] = 0.0;

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

    array.data[0] = array.data[0]+joy_pose.position.x;
    array.data[1] = array.data[1]+joy_pose.position.y;
    array.data[2] = array.data[2]+joy_pose.position.z;

    array.data[3] = joy_pose.orientation.x;
    array.data[4] = joy_pose.orientation.y;
    array.data[5] = joy_pose.orientation.z;
    array.data[6] = joy_pose.orientation.w;
    
    array.data[7] = array.data[7];
    array.data[8] = array.data[8];
    array.data[9] = array.data[9];

    target_pose.header.frame_id="base_link";
    target_pose.pose.position.x=array.data[0];
    target_pose.pose.position.y=array.data[1];
    target_pose.pose.position.z=array.data[2];

    target_pose.pose.orientation.x=array.data[3];
    target_pose.pose.orientation.y=array.data[4];
    target_pose.pose.orientation.z=array.data[5];
    target_pose.pose.orientation.w=array.data[6];
    pub.publish(array);
    pub1.publish(target_pose);
    ROS_INFO_STREAM("target_frame"<<target_pose);
    ROS_INFO_STREAM("target_force\n"<<"x="<<array.data[7]<<",y="<<array.data[8]<<",z="<<array.data[9]);
  }

void Frame_pub::callback(const sensor_msgs::Joy::ConstPtr& data){
    joy_pose.position.z= (data->axes[1])*0.0003;
    joy_pose.position.x= (data->axes[7])*0.0003;
    joy_pose.position.y= (data->axes[0])*0.0003;
    geometry_msgs::Point diff=orientation_count((data->axes[4])*0.5,(data->axes[6])*0.5,(data->axes[3])*0.5);
    tf::Quaternion joy_quat=rpy_to_tf_quat(-90+diff.x,0+diff.y,0+diff.z);
    joy_pose.orientation.x=joy_quat.getX();
    joy_pose.orientation.y=joy_quat.getY();
    joy_pose.orientation.z=joy_quat.getZ();
    joy_pose.orientation.w=joy_quat.getW();
    if(data->buttons[0]==0){
    array.data[7]=array.data[7]+data->buttons[2]*1;
    array.data[8]=array.data[8]+data->buttons[3]*1;
    array.data[9]=array.data[9]+data->buttons[1]*1;
    }
    else
    {
    array.data[7]=array.data[7]-data->buttons[2]*1;
    array.data[8]=array.data[8]-data->buttons[3]*1;
    array.data[9]=array.data[9]-data->buttons[1]*1;
    }

    if(data->buttons[9]==1){
      frame_pub();
    }
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