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
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std;

class pos_force_controller{

  public:
    pos_force_controller();
    void start_pid();

  private:
    ros::NodeHandle nh;

    ros::Subscriber sub;
    ros::Subscriber current_force_sub;
    ros::Publisher pub;
    ros::Publisher pub_2;

    ros::Timer timer_;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;

    vector<double> target_orientation{4,0};
    std_msgs::Header  world_frame;

    vector<double> target_pose{3,0};
    vector<double> current_pose{3,0};
    vector<double> previous_pose{3,0};
    vector<vector<double>> pose_error{{0,0},{0,0},{0,0}};
    // vector<double> default_force_sd{0.0008,1.5};////{Spring,Damper,D}
    vector<double> default_force_sd{0.002,995000.5};////{Spring,Damper,D}

    // vector<double> default_force_sd{0.0005*1.8,0.};////{Spring,Damper,D}

    vector<double> force_sd{0.0,0.0};//{Spring,Damper,D}

    vector<double> target_force{3,0};
    vector<double> current_force{3,0};


    vector<double> target_torque{3,0};
    vector<double> current_torque{3,0};
    // vector<double> default_torque_sd{80*0.1,60.0*0.1};////{Spring,Damper,D}
    vector<double> default_torque_sd{0.0,0.0};////{Spring,Damper,D}

    vector<double> torque_sd{0.0,0.0};//{Spring,Damper,D}


    geometry_msgs::PoseStamped send_frame;
    geometry_msgs::Point rpy;

    void frame_pub(const vector<double> &pose,const vector<double> &torque);
    void callback(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void current_force_callback(const geometry_msgs::WrenchStampedConstPtr& msg);


    vector<double> force_spring_damp_ctrl(
                                          const vector<double> &target_force,
                                          const vector<double> &current_force);


    vector<double> torque_spring_damp_ctrl(const vector<double> &current_pose,
                                          const vector<double> &target_torque,
                                          const vector<double> &current_torque);


    //utils
    tf::Quaternion rpy_to_tf_quat(double roll, double pitch, double yaw);
    vector<double>  check_outlines(const vector<double>& data ,double max, double min);

};

pos_force_controller::pos_force_controller():nh(), tfBuffer_(), tfListener_(tfBuffer_)
{
  pub_2 = nh.advertise<geometry_msgs::Point>("/end_effector_point", 1); // for visualization

  // pose_pid_gain   = default_pose_pid_gain;
  // force_pid_gain  = default_force_pid_gain;
  // torque_pid_gain = default_torque_pid_gain;

  force_sd = default_force_sd;
  torque_sd = default_torque_sd;

  target_pose[0] = -0.119;
  target_pose[1] = 0.493;
  target_pose[2] = 0.5;

  previous_pose[0] = -0.119;
  previous_pose[1] = 0.493;
  previous_pose[2] = 0.5;



  tf2::Quaternion q;
  q.setRPY((-90)*(M_PI/180), 0*(M_PI/180),0*(M_PI*180));
  target_orientation[0] = q.x();
  target_orientation[1] = q.y();
  target_orientation[2] = q.z();
  target_orientation[3] = q.w();

  timer_ = nh.createTimer(ros::Duration(0.002), [&](const ros::TimerEvent& e) {
  geometry_msgs::TransformStamped transformStamped;
  geometry_msgs::TransformStamped force_transform;
    try{
      transformStamped = tfBuffer_.lookupTransform("base_link", "tool0", ros::Time(0));
      force_transform = tfBuffer_.lookupTransform("base_link", "force_link", ros::Time(0));
      }
    catch (tf2::TransformException& ex){
      ROS_WARN("error->%s", ex.what());
      return;
    }

    //copy to geometry_msgs::PoseStamped current_pose
    world_frame.frame_id=transformStamped.header.frame_id;
    current_pose[0]=transformStamped.transform.translation.x;
    current_pose[1]=transformStamped.transform.translation.y;
    current_pose[2]=transformStamped.transform.translation.z;
    // ROS_INFO_STREAM("world->ee_link.trans:"<<transformStamped.transform.rotation);
    //ROS_INFO_STREAM("headr_frame_id x= "<<world_frame.frame_id);
    ROS_INFO_STREAM("current_pose x= "<<current_pose[0]<<", y="<<current_pose[1]<<", z= "<<current_pose[2]);

    vector<double> temp{3,0};
    temp[0]=force_transform.transform.translation.x;
    temp[1]=force_transform.transform.translation.y;
    temp[2]=force_transform.transform.translation.z;
    current_force=check_outlines(temp,50.0,-50.0);
    ROS_INFO_STREAM("current_force x= "<<current_force[0]<<", y="<<current_force[1]<<", z= "<<current_force[2]);
    geometry_msgs::Point pout;
    pout.x =current_pose[0];
    pout.y =current_pose[1];
    pout.z =current_pose[2];
    pub_2.publish(pout);
    });

  current_force_sub = nh.subscribe("/LowPass_filtered_wrench",10,&pos_force_controller::current_force_callback,this);//subscribe ft_sensor
  sub = nh.subscribe("array", 1, &pos_force_controller::callback,this); //subscribe input data
  pub = nh.advertise<geometry_msgs::PoseStamped>("/end_effector_pose", 1); //pub frame to ik solver node

}




tf::Quaternion pos_force_controller::rpy_to_tf_quat(double roll, double pitch, double yaw){
  /*************************
  input:roll,pitch,yaw
  output: quaternion x,y,z,w
  **************************/
  // ROS_INFO_STREAM("rpy-"<<rpy.x<<rpy.y<<rpy.z);
  return tf::createQuaternionFromRPY(rpy.x, rpy.y, rpy.z);
}



//sub wrench torque
void pos_force_controller::current_force_callback(const geometry_msgs::WrenchStampedConstPtr& msg){
  current_torque[0]  = msg->wrench.torque.x;
  current_torque[1]  = msg->wrench.torque.y;
  current_torque[2]  = msg->wrench.torque.z;
}

vector<double> pos_force_controller::check_outlines(const vector<double> &data, double max,double min){
vector<double>  val{3,0};
  for(uint i=0;i<=2;i++){
    if (max<=data[i])
    {
      val[i]=max;
    }
    else if (data[i]<=min)
    {
      val[i]=min;
    }
    else
    {
      val[i]=data[i];
    }
    }
    return val;
  }

//pub frame to ik solver node
void pos_force_controller::frame_pub(const vector<double> &pose ,const vector<double> &torque) {

    vector<double> final_torque{3,0};
    vector<double> final_orientation{4,0};

    send_frame.header.frame_id=world_frame.frame_id;

    // send_frame.pose.position.x=current_pose[0]+pose[0];
    // send_frame.pose.position.y=current_pose[1]+pose[1];
    // send_frame.pose.position.z =current_pose[2]+pose[2];


    send_frame.pose.position.x =  target_pose[0]  + pose[0];
    send_frame.pose.position.y =  target_pose[1]  + pose[1];
    send_frame.pose.position.z =  target_pose[2]  + pose[2];

    // if (target_pose[1]+pose[1]>target_pose[1])
    //   {
    //   send_frame.pose.position.y = target_pose[1]-target_pose[1]*0.0001;
    //   }

    if (-0.0005<pose[1] && pose[1]<0.0005)
      {
        send_frame.pose.position.y = target_pose[1];
      }



    if (-0.0005<pose[2] && pose[2]<0.0005)
      {
        send_frame.pose.position.z = target_pose[2];
      }



    if (-0.0005<pose[0] && pose[0]<0.0005)
      {
        send_frame.pose.position.x = target_pose[0];
      }




      tf::Quaternion q2(
          target_orientation[0],
          target_orientation[1],
          target_orientation[2],
          target_orientation[3]);

      tf::Matrix3x3 m(q2);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);

      final_torque[0] = roll  - torque[0] * 10.;
      final_torque[1] = pitch - torque[2] * 10.; // z
      final_torque[2] = yaw   + torque[1] * 10.;   // y

      ROS_INFO_STREAM("pub_frame"
                      << "x=" << torque[0] * 5 << " y= " << torque[1] * 5 << " z= " << torque[2] * 5);



      tf2::Quaternion q1;
      q1.setRPY(final_torque[0], final_torque[1], final_torque[2]);



      send_frame.pose.orientation.x = q1.x();
      send_frame.pose.orientation.y = q1.y();
      send_frame.pose.orientation.z = q1.z();
      send_frame.pose.orientation.w = q1.w();

      pub.publish(send_frame);
      // ROS_INFO_STREAM("pub_frame"<<"x="<<send_frame.pose.position.x<<" y= "<<send_frame.pose.position.y<<" z= "<<send_frame.pose.position.z);
      ROS_INFO_STREAM("target_force x= " << send_frame << "f");
}
//sub target value
void pos_force_controller::callback(const std_msgs::Float32MultiArray::ConstPtr& msg){
    if(msg->data.size()!=12){
      ROS_ERROR("exception error. the size of input data must be 12");
      exit(EXIT_FAILURE);
    }

    // pose_pid_gain = default_pose_pid_gain;
    // force_pid_gain = default_force_pid_gain;
    // torque_pid_gain = default_torque_pid_gain;

    force_sd = default_force_sd;
    torque_sd = default_torque_sd;

    target_pose[0]=(msg->data[0]);
    target_pose[1]=(msg->data[1]);
    target_pose[2]=(msg->data[2]);
    target_orientation[0]=msg->data[3];
    target_orientation[1]=msg->data[4];
    target_orientation[2]=msg->data[5];
    target_orientation[3]=msg->data[6];
    target_force[0]=msg->data[7];
    target_force[1]=msg->data[8];
    target_force[2]=msg->data[9];
    if(msg->data[10]==0){
      force_sd[0]=0.0;
      force_sd[1]=0.0;
    }
    if(msg->data[11]==0){
      torque_sd[0]=0.0;
      torque_sd[1]=0.0;
    }
    // ROS_INFO_STREAM("target_pose x= "<<target_pose[0]<<", y="<<target_pose[1]<<", z= "<<target_pose[2]);
    // ROS_INFO_STREAM("target_force x= "<<target_force[0]<<", y="<<target_force[1]<<", z= "<<target_force[2]);
}




vector<double> pos_force_controller::force_spring_damp_ctrl(const vector<double> &target_force,
                                      const vector<double> &current_force){

    vector<double> force_result{3,0.0};
    double position;
    double velocity;

    double damper;
    double spring;

    spring = force_sd[0];
    damper = force_sd[1];

    for (uint i=0;i<=force_result.size();i++){
    // for (uint i=0;i<=1;i++){

      position = -current_force[i]+target_force[i];

      velocity = (pose_error[i][1]-pose_error[i][0])/2*0.005;

      // velocity = (current_pose[i]-previous_pose[i])/2*0.004;

      previous_pose[i] = current_pose[i];

      force_result[i] = damper*velocity+spring*position;
    }



    return force_result;
                                      }



vector<double> pos_force_controller::torque_spring_damp_ctrl(const vector<double> &current_pose,
                                      const vector<double> &target_torque,
                                      const vector<double> &current_torque){

    vector<double> force_result{3,0.0};
    double position;
    double velocity;

    double damper;
    double spring;

    spring = torque_sd[0];
    damper = torque_sd[1];

    for (uint i=0;i<=force_result.size();i++){

      position = -current_torque[i]+target_torque[i];
      velocity = (current_pose[i]-previous_pose[i])/0.002;
      previous_pose[i] = current_pose[i];

      force_result[i] = damper*velocity+spring*position;
    }

    return force_result;
                                      }









//main loop
void pos_force_controller::start_pid(){

  vector<double> force_pid_result{3,0};
  vector<double> torque_pid_result{3,0};
  vector<double> hole_pid_result{3,0};

  force_pid_result  = force_spring_damp_ctrl(target_force,current_force);


  target_torque[0]=0.0;
  target_torque[1]=0.0;
  target_torque[2]=0.0;



  torque_pid_result = torque_spring_damp_ctrl(current_pose,target_torque,current_torque);
  ROS_INFO_STREAM("hole_pid_result "<<"x="<<torque_pid_result[0]<<",y="<<torque_pid_result[1]<<",z="<<torque_pid_result[2]);


  for (uint i=0;i<=2;i++){
    hole_pid_result[i] = -force_pid_result[i];
  }
  frame_pub(hole_pid_result,torque_pid_result);
}

int main( int argc, char** argv )
{

  ros::init(argc, argv, "pos_force_pid_controller");

  pos_force_controller my1;
  //ros::Duration(2).sleep();
  ros::Rate loop_rate(500);//500Hz
  while (ros::ok())
  {
    my1.start_pid();
    ros::spinOnce();
    loop_rate.sleep();
  }

}

        // force_x = data.wrench.force.x*(force_threshold>data.wrench.force.x>-force_threshold)
        // force_y = data.wrench.force.y*(force_threshold>data.wrench.force.y>-force_threshold)
        // force_z = data.wrench.force.z*(force_threshold>data.wrench.force.z>-force_threshold)

        // torque_x = data.wrench.torque.x*(torque_threshold>data.wrench.torque.x>-torque_threshold)
        // torque_y = data.wrench.torque.y*(torque_threshold>data.wrench.torque.y>-torque_threshold)
        // torque_z = data.wrench.torque.z*(torque_threshold>data.wrench.torque.z>-torque_threshold)