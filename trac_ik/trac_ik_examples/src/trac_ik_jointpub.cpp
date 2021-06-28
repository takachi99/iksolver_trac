#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/frames_io.hpp>

#include <trac_ik/trac_ik.hpp>

#include <stdio.h>
#include <iostream>

 #include <ros/ros.h>
 #include <sensor_msgs/JointState.h>
 #include <std_msgs/Header.h>
 #include <trajectory_msgs/JointTrajectory.h>
 #include <geometry_msgs/PoseStamped.h>

 #include <tf/transform_broadcaster.h>

#include <cmath>

class My_joint_pub{

    public:
    My_joint_pub();
    void callback(const geometry_msgs::PoseStamped::ConstPtr& data);
    void Publication(const KDL::JntArray &result );
    double eps,timeout;
    std::string chain_start, chain_end, urdf_param;
    
    KDL::JntArray pre_result;
  private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pub;
    void find_IK(const KDL::Frame &end_effector_pose);
    double pre_joyval=0;

     };

My_joint_pub::My_joint_pub()
{
  nh.param("urdf_param", urdf_param, std::string("/robot_description"));
  chain_start = "base_link";
  chain_end = "ee_link";
  timeout = 0.002;
  eps = 1e-4;
  pub = nh.advertise<trajectory_msgs::JointTrajectory>("arm_controller/command", 10);
  sub = nh.subscribe("end_effector_pose", 10, &My_joint_pub::callback,this);
  pre_result.resize(6);
  // ros::Rate loop_rate(500);
}

void My_joint_pub::find_IK(const KDL::Frame &end_effector_pose)
  {
    TRAC_IK::TRAC_IK tracik_solver(chain_start, chain_end, urdf_param, timeout, eps);
    
    KDL::Chain chain;
    KDL::JntArray ll, ul; //lower joint limits, upper joint limits

  bool valid = tracik_solver.getKDLChain(chain);

    if (!valid)
    {
      ROS_ERROR("There was no valid KDL chain found");
      return;
    }

    valid = tracik_solver.getKDLLimits(ll, ul); //calc joint limit from urdf model
    if (!valid)
    {
      ROS_ERROR("There were no valid KDL joint limits found");
      return;
    }
  
  //calc joint limit own setting
  for (uint j = 0; j < ll.data.size(); j++)
  {
    ll(j) = -3.14 / 2.0;
    ul(j) = 3.14 / 2.0;
  }

  assert(chain.getNrOfJoints() == ll.data.size());
  assert(chain.getNrOfJoints() == ul.data.size());


  // ROS_INFO("Using %d joints!", chain.getNrOfJoints());
  // for(unsigned int i=0; i<chain.getNrOfJoints(); i++)
	// {
	// 	ROS_INFO("joint_name[%d]: %s", i, chain.getSegment(i).getJoint().getName().c_str());
	// 	ROS_INFO_STREAM("lower_joint_limits:"<<ll.data(i,0));//chain_info.joint_names.push_back(chain.getSegment(i).getJoint().getName());
  //   ROS_INFO_STREAM("upper_joint_limits:"<<ul.data(i,0));
  // }

  // Create Nominal chain configuration midway between all joint limits
  KDL::JntArray nominal(chain.getNrOfJoints());

  for (uint j = 0; j < nominal.data.size(); j++)
  {
    nominal(j) = (ll(j) + ul(j)) / 2.0;
  }
   KDL::JntArray result(ll.data.size());
  int rc=0;
  
  rc = tracik_solver.CartToJnt(nominal, end_effector_pose, result);
  ROS_INFO_STREAM("trac ik result="<<rc);
  for (uint i=0;i<result.data.size();i++){
  ROS_INFO_STREAM("IK_result.data("<<i<<",0)="<<result.data(i,0));
    }
  //   if(rc>0){
  //   trajectory_msgs::JointTrajectory tr0;
  //   tr0.header.frame_id ="base_link";
  //   tr0.joint_names.resize(6);
  //   tr0.points.resize(1);
  //   tr0.points[0].positions.resize(6);
  //   tr0.header.stamp = ros::Time::now();
  //   tr0.points[0].time_from_start = ros::Duration(0.2);
  // for(unsigned int i=0; i<chain.getNrOfJoints(); i++)
	// {
  //   tr0.joint_names[i] = chain.getSegment(i).getJoint().getName().c_str();
  //   tr0.points[0].positions[i]=result(i);
  // }
  // pub.publish(tr0);
  //   // ros::Rate loop_rate(0.002);
  //   //  loop_rate.sleep();
  //   }

  if(rc>0){
    double current_joyval=0;
    for (uint i=0;i<result.data.size();i++){
      current_joyval  = current_joyval + result.data(i,0);
      //pre_joyval= pre_joyval+pre_result.data(i,0);
    }
    ROS_INFO_STREAM("current_joint sum"<<current_joyval);
    ROS_INFO_STREAM("pre_joint sum"<<pre_joyval);
    ROS_INFO_STREAM("joint diff"<<std::abs(std::fabs(current_joyval)-std::fabs(pre_joyval)));
    trajectory_msgs::JointTrajectory tr0;
    tr0.header.frame_id ="base_link";
    tr0.joint_names.resize(6);
    tr0.points.resize(1);
    tr0.points[0].positions.resize(6);
    tr0.header.stamp = ros::Time::now();
    tr0.points[0].time_from_start = ros::Duration(0.02);

    if (std::abs(std::fabs(current_joyval)-std::fabs(pre_joyval))){

      for(unsigned int i=0; i<chain.getNrOfJoints(); i++)
      {
      tr0.joint_names[i] = chain.getSegment(i).getJoint().getName().c_str();
      tr0.points[0].positions[i]=result(i);
      pre_result.data(i,0)=result(i);
      }
    pre_joyval=current_joyval;
   
    }
    else{
      for(unsigned int i=0; i<chain.getNrOfJoints(); i++)
      {
      tr0.joint_names[i] = chain.getSegment(i).getJoint().getName().c_str();
      tr0.points[0].positions[i]=pre_result(i);
            }
    }
     pub.publish(tr0);
  }


  }


void My_joint_pub::callback(const geometry_msgs::PoseStamped::ConstPtr& data) {
  // ROS_INFO("Callbacking...");
  KDL::Vector end_pos(data->pose.position.x,data->pose.position.y,data->pose.position.z);
  KDL::Rotation end_rot=KDL::Rotation::Quaternion(data->pose.orientation.x,data->pose.orientation.y,data->pose.orientation.z,data->pose.orientation.w);
  KDL::Frame end_effector_pose(end_rot,end_pos);
  find_IK(end_effector_pose);
  }

void My_joint_pub::Publication(const KDL::JntArray &result){

}



int main( int argc, char** argv )
{

ros::init(argc, argv, "talker");

My_joint_pub my1;
// trajectory_msgs::JointTrajectory tr0;
// tr0.header.frame_id ="base_link";
// tr0.joint_names.resize(6);
// tr0.points.resize(1);

// tr0.joint_names[0] = "shoulder_pan_joint";
// tr0.joint_names[1] = "shoulder_lift_joint";
// tr0.joint_names[2] = "elbow_joint";
// tr0.joint_names[3] ="wrist_1_joint";
// tr0.joint_names[4] ="wrist_2_joint";
// tr0.joint_names[5] = "wrist_3_joint";

// tr0.points[0].positions.resize(6);
// tr0.points[0].positions[0]=0;
// tr0.points[0].positions[1]=-1.8;
// tr0.points[0].positions[2]=0.66;
// tr0.points[0].positions[3]=1.14;
// tr0.points[0].positions[4]=1.56;
// tr0.points[0].positions[5]=-3.14;

  ros::Rate loop_rate(500);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

}



// #include <ros/ros.h>
// #include <std_msgs/String.h>

// class RosWithClass{
// 	private:
// 		ros::NodeHandle nh;
// 		ros::Publisher pub;
// 		ros::Subscriber sub;
// 		std_msgs::String pub_msg;
// 	public:
// 		RosWithClass();
// 		void Publication(void);
// 		void Callback(const std_msgs::String::ConstPtr& msg);
// };

// RosWithClass::RosWithClass()
// {
// 	pub = nh.advertise<std_msgs::String>("/pub_msg", 1);
// 	sub = nh.subscribe("/message", 1, &RosWithClass::Callback, this);
// }

// void RosWithClass::Publication(void)
// {
// 	pub_msg.data = "message";
// 	pub.publish(pub_msg);
// }

// void RosWithClass::Callback(const std_msgs::String::ConstPtr& msg)
// {
// 	std::cout << msg->data.c_str() << std::endl;
// }

// int main(int argc, char**argv)
// {
// 	ros::init(argc, argv, "talker");
	
// 	RosWithClass ros_with_class;

// 	ros::Rate loop_rate(10);
// 	while(ros::ok()){
// 		ros_with_class.Publication();
// 		ros::spinOnce();
// 		loop_rate.sleep();
// 	}
// }