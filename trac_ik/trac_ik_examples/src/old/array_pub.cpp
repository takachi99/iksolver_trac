#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "basic_array_talker");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<std_msgs::Float32MultiArray>("array", 10);

  ros::Rate loop_rate(500);
  while (ros::ok())
  {
    std_msgs::Float32MultiArray array;
    array.data.resize(12);
    array.data[0] = -0.116;
    array.data[1] = 0.493;
    array.data[2] = 0.5;
    tf2::Quaternion q;
    q.setRPY((-90)*(M_PI/180), 0*(M_PI/180),0*(M_PI*180));
    array.data[3] = q.x();
    array.data[4] = q.y();
    array.data[5] = q.z();
    array.data[6] = q.w();

    array.data[7] = 0.0;
    array.data[8] = 0.0;
    array.data[9] = 1.0;
    array.data[10] = 1.0;
    array.data[11] = 1.0;

    pub.publish(array);

  ROS_INFO_STREAM("I published array!"<<array);
    ros::spinOnce();
    loop_rate.sleep();
  }
}