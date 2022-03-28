

#include "ros/ros.h"
# include "std_srvs/Trigger.h"




bool add(std_srvs::Trigger::Request  &req,
         std_srvs::Trigger::Response &res)
{
//   res.sum = req.a + req.b;
//   ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
//   ROS_INFO("sending back response: [%ld]", (long int)res.sum);
    ROS_INFO_STREAM("target_pose x= "<<req<<"hogeee");
    return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("add_two_ints", add);
  ROS_INFO("Ready to add two ints.");
  ros::spin();

  return 0;
}