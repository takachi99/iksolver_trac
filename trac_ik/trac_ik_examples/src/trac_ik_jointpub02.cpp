/********************************************************************************
Copyright (c) 2016, TRACLabs, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
 are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice,
       this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.

    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software
       without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************************/

#include <boost/date_time.hpp>
#include <trac_ik/trac_ik.hpp>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <kdl/chainiksolverpos_nr_jl.hpp>



double fRand(double min, double max)
{
  double f = (double)rand() / RAND_MAX;
  return min + f * (max - min);
}


void test(ros::NodeHandle& nh, double num_samples, std::string chain_start, std::string chain_end, double timeout, std::string urdf_param)
{

  double eps = 1e-6;

  // This constructor parses the URDF loaded in rosparm urdf_param into the
  // needed KDL structures.  We then pull these out to compare against the KDL
  // IK solver.
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
  // for (uint j = 0; j < ll.data.size(); j++)
  // {
  //   ll(j) = -3.14 / 2.0;
  //   ul(j) = 3.14 / 2.0;
  // }

  assert(chain.getNrOfJoints() == ll.data.size());
  assert(chain.getNrOfJoints() == ul.data.size());


  ROS_INFO("Using %d joints!", chain.getNrOfJoints());
  for(unsigned int i=0; i<chain.getNrOfJoints(); i++)
	{
		ROS_INFO("joint_name[%d]: %s", i, chain.getSegment(i).getJoint().getName().c_str());
		ROS_INFO_STREAM("lower_joint_limits:"<<ll.data(i,0));//chain_info.joint_names.push_back(chain.getSegment(i).getJoint().getName());
    ROS_INFO_STREAM("upper_joint_limits:"<<ul.data(i,0));
  }


  // Set up KDL IK
  KDL::ChainFkSolverPos_recursive fk_solver(chain); // Forward kin. solver
  KDL::ChainIkSolverVel_pinv vik_solver(chain); // PseudoInverse vel solver
  KDL::ChainIkSolverPos_NR_JL kdl_solver(chain, ll, ul, fk_solver, vik_solver, 1, eps); // Joint Limit Solver
  // 1 iteration per solve (will wrap in timed loop to compare with TRAC-IK)


  // Create Nominal chain configuration midway between all joint limits
  KDL::JntArray nominal(chain.getNrOfJoints());

  for (uint j = 0; j < nominal.data.size(); j++)
  {
    nominal(j) = (ll(j) + ul(j)) / 2.0;
  }

  // Create inital jointArray
  std::vector<KDL::JntArray> JointList;
  KDL::JntArray q(chain.getNrOfJoints());

double joint[]={1.5,0,-1.98,0.0,-1.7,-1.57,1.43};
    for (uint j = 0; j < ll.data.size(); j++)
    {
      //q(j) = fRand(ll(j), ul(j));
      //q(j) = joint[j];
       q(j) = 0.1;
    }
    JointList.push_back(q);


  ROS_INFO_STREAM("JointList.data.size="<<JointList[0].data.size());
  ROS_INFO_STREAM("JointList.data_matrix.size()="<<JointList[0].data.rows()<<"x"<<JointList[0].data.cols());
  for (uint i=0;i<JointList[0].data.size();i++){
    std::cout<<"JointList[0].data("<<i<<",0)="<<JointList[0].data(i,0)<<std::endl;
  }


  boost::posix_time::ptime start_time;
  boost::posix_time::time_duration diff;

  KDL::JntArray result(JointList[0].data.size());

/*
ur
- Translation: [0.256, 0.109, 0.714]
- Rotation: in Quaternion [-0.327, 0.327, -0.627, 0.627]
            in RPY (radian) [-0.962, 0.001, -1.570]
            in RPY (degree) [-55.093, 0.037, -89.959]
*/
  //KDL::Rotation end_rot=KDL::Rotation::RPY(-0.0001,0.001,-1.570); //
  KDL::Rotation end_rot=KDL::Rotation::Quaternion(-0.00037397335412140656, 0.0001904047369388022, -0.7071646747715623, 0.7070487583223644); //

  KDL::Vector end_pos(0.063,0.109,0.764);
  KDL::Frame end_effector_pose(end_rot,end_pos);
  // KDL::Vector rot=end_effector_pose.M.GetRot();
  // std::cout << "endeffector.pose.M.GetRot().x="<<rot.x()<<std::endl;
  // std::cout << "endeffector.pose.M.GetRot().y="<<rot.y()<<std::endl;
  // std::cout << "endeffector.pose.M.Getrot().z="<<rot.z()<<std::endl;
  // std::cout << "end_effector_pose.p.x()="<<end_effector_pose.p.x()<<std::endl;
  // std::cout << "end_effector_pose.p.y()="<<end_effector_pose.p.y()<<std::endl;
  // std::cout << "end_effector_pose.p.z()="<<end_effector_pose.p.z()<<std::endl;


  // ROS_INFO_STREAM("KDL ik start");
  // int rc;
  // fk_solver.JntToCart(q1, end_effector_pose);
  // rc = kdl_solver.CartToJnt(q1, end_effector_pose, result);
  // ROS_INFO_STREAM("KDL ik result="<<rc);
  // for (uint i=0;i<result.data.size();i++){
  //   ROS_INFO_STREAM("JointListdata("<<i<<",0)="<<result.data(i,0));
  // }



  int rc=0;
  //fk_solver.JntToCart(j1, end_effector_pose);
  rc = tracik_solver.CartToJnt(nominal, end_effector_pose, result);
  ROS_INFO_STREAM("trac ik result="<<rc);
  for (uint i=0;i<result.data.size();i++){
  ROS_INFO_STREAM("IK_result.data("<<i<<",0)="<<result.data(i,0));
  }



}



int main(int argc, char** argv)
{
  srand(1);
  ros::init(argc, argv, "ik_tests");
  ros::NodeHandle nh("~");

  int num_samples;
  std::string chain_start, chain_end, urdf_param;
  double timeout;

  //nh.param("num_samples", num_samples, 1000);
  //nh.param("chain_start", chain_start, std::string(""));
  //nh.param("chain_end", chain_end, std::string(""));
  //nh.param("timeout", timeout, 0.005);

  nh.param("urdf_param", urdf_param, std::string("/robot_description"));
  num_samples =1000;
  //chain_start = "torso_lift_link";
  //chain_end = "r_wrist_roll_link";

  chain_start = "base_link";
  chain_end = "ee_link";
  timeout = 0.005;


  if (chain_start == "" || chain_end == "")
  {
    ROS_FATAL("Missing chain info in launch file");
    exit(-1);
  }

  if (num_samples < 1)
    num_samples = 1;

  test(nh, num_samples, chain_start, chain_end, timeout, urdf_param);

  // Useful when you make a script that loops over multiple launch files that test different robot chains
  // std::vector<char *> commandVector;
  // commandVector.push_back((char*)"killall");
  // commandVector.push_back((char*)"-9");
  // commandVector.push_back((char*)"roslaunch");
  // commandVector.push_back(NULL);

  // char **command = &commandVector[0];
  // execvp(command[0],command);
  return 0;
}
