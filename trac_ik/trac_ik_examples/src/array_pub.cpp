#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "basic_array_talker");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<std_msgs::Float32MultiArray>("array", 10);

  ros::Rate loop_rate(500);
  while (ros::ok())
  {
    std_msgs::Float32MultiArray array;
    array.data.resize(10);
    array.data[0] = 0.36;
    array.data[1] = 0.109;
    array.data[2] = 0.5;

    array.data[3] = 0.0;
    array.data[4] = 0.0;
    array.data[5] = 0.0;
    array.data[6] = 1.0;
    
    array.data[7] = 3.0;
    array.data[8] = 0.0;
    array.data[9] = 1.0;
    pub.publish(array);
  ROS_INFO_STREAM("I published array!"<<array);
    ros::spinOnce();
    loop_rate.sleep();
  }
}