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
  private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pub;
    ros::Publisher pub1;

    geometry_msgs::Pose joy_pose;
    geometry_msgs::PoseStamped target_pose;

     };

Frame_pub::Frame_pub()

{   sub = nh.subscribe("joy", 10, &Frame_pub::callback,this);
    pub = nh.advertise<std_msgs::Float32MultiArray>("/array", 1);
    pub1 = nh.advertise<geometry_msgs::PoseStamped>("target_frame",1);
    array.data.resize(10);
    array.data[0] = 0.36;//x
    array.data[1] = 0.109;//y
    array.data[2] = 0.5;//z

    array.data[3] = 0.0;
    array.data[4] = 0.0;
    array.data[5] = 0.0;
    array.data[6] = 1.0;

    array.data[7] = 0.0;
    array.data[8] = 0.0;
    array.data[9] = 0.0;

}
tf::Quaternion Frame_pub::rpy_to_tf_quat(double roll, double pitch, double yaw){
  rpy.x=rpy.x+roll;
  rpy.y=rpy.y+pitch;
  rpy.z=rpy.z+yaw;
  // ROS_INFO_STREAM("r="<<rpy.x<<",p="<<rpy.y<<",z"<<rpy.z);
    return tf::createQuaternionFromRPY(rpy.x, rpy.y, rpy.z);
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
    joy_pose.position.z= int(data->axes[1])*0.0001;
    joy_pose.position.x= (-1.0*data->axes[0])*0.0001;
    joy_pose.position.y= (1.0*data->axes[7])*0.0001;
    tf::Quaternion joy_quat=rpy_to_tf_quat(int(data->axes[3])*0.05,int(data->axes[4])*0.05,int(data->axes[6])*0.05);
    joy_pose.orientation.x=joy_quat.getX();
    joy_pose.orientation.y=joy_quat.getY();
    joy_pose.orientation.z=joy_quat.getZ();
    joy_pose.orientation.w=joy_quat.getW();
    if(data->buttons[0]==0){
    array.data[7]=array.data[7]+data->buttons[2]*0.01;
    array.data[8]=array.data[8]+data->buttons[3]*0.01;
    array.data[9]=array.data[9]+data->buttons[1]*0.01;
    }
    else
    {
    array.data[7]=array.data[7]-data->buttons[2]*0.01;
    array.data[8]=array.data[8]-data->buttons[3]*0.01;
    array.data[9]=array.data[9]-data->buttons[1]*0.01;
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