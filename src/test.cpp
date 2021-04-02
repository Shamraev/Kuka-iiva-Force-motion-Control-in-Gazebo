
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/treejnttojacsolver.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include "Eigen/Eigen"
#include "Eigen/Geometry"
#include <string>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/WrenchStamped.h"
#include <controller_manager_msgs/SwitchController.h>

#define dt 0.01

// Parameters for PD-controller with gravity compensation
std::vector<float> KP = {50.0, 150.0, 40.0, 20.0, 18.0, 18.0, 15.0};
std::vector<float> KD = {5.0, 25.0, 5.0, 2.0, 1.0, 1.0, 0.5};

KDL::JntArray q(7);
KDL::JntArray dq(7);
KDL::JntArray ddq(7);
KDL::JntArray meas_torques(7);
KDL::JntArray grav_torques(7);
std::array<ros::Publisher,7> torque_controller_pub;

void iiwaStateCallback(const sensor_msgs::JointState msg);
void startControllers(ros::NodeHandle & nh);

int main(int argc, char **argv) {
  ros::init(argc, argv, "main_test");
  ros::NodeHandle nh;
  ros::Subscriber t = nh.subscribe("/iiwa/joint_states", 1000, iiwaStateCallback);

  // Initialize publishers for send computation torques.
  for(int i =0; i<7; i++){
      torque_controller_pub.at(i) = nh.advertise<std_msgs::Float64>("/iiwa/joint" + std::to_string(i+1) +"_torque_controller/command", 1000);
  }
  ros::Rate loop_rate((int)(1.0/dt));

  // Get parameters for kinematic from setup.yaml file
  std::string urdf;
  std::string base_link;
  std::string tool_link;
  float force_desired = 0;
  ros::param::get("/profi2021_master_solution/base_link_name", base_link);
  ros::param::get("/profi2021_master_solution/tool_link_name", tool_link);
  ros::param::get("iiwa_urdf_model", urdf);
  ros::param::get("/profi2021_master_solution/force_desired", force_desired);

  ROS_INFO("Roslaunch param desired force:  %f", force_desired);

  // Generate kinematic model for orocos_kdl
  KDL::Tree tree;
  if (!kdl_parser::treeFromString(urdf, tree)) {
      printf("Failed to construct kdl tree\n");
      return -1;
  }
  KDL::Chain chain;
  if(!tree.getChain(base_link, tool_link, chain)){
      ROS_ERROR_STREAM("Failed to get KDL chain from tree ");
      return false;
  }
  ROS_INFO("tip_name:  %s",tool_link.c_str());
  ROS_INFO("root_name: %s",base_link.c_str());

  Eigen::Matrix<double,6,1> L;
  L(0)=1;L(1)=1;L(2)=1;L(3)=0.01;L(4)=0.01;L(5)=0.01;
  KDL::ChainIkSolverPos_LMA iksolverpos(chain,L);

  // Generate dynamic model for orocos_kdl
  KDL::Vector grav(0, 0, -9.82);
  KDL::ChainDynParam dyn_model(chain, grav);

  startControllers(nh);

  std_msgs::Float64 torq_msg;
  uint16_t counter = 0;
  KDL::JntArray u(7);
  KDL::JntArray q_dest(7);
  KDL::Frame frame_dest;

  // Setting up initial point
  frame_dest.p(0) = 0.4;
  frame_dest.p(1) = 0.0;
  frame_dest.p(2) = 0.1;
  frame_dest.M = KDL::Rotation::RotY(3.14);
  int ret = iksolverpos.CartToJnt(q,frame_dest,q_dest);

  while (ros::ok()){
      if (counter*dt>3.0){
          frame_dest.p(0) = 0.1*sin(counter*dt) + 0.5;
          frame_dest.p(1) = 0.1*cos(counter*dt);
      }
      ret = iksolverpos.CartToJnt(q,frame_dest,q_dest);
      ROS_INFO("IK solution: %f, %f, %f, %f, %f, %f, %f", q_dest(0),  q_dest(1),  q_dest(2),  q_dest(3),  q_dest(4),  q_dest(5),  q_dest(6));
      dyn_model.JntToGravity(q, grav_torques);
      ROS_INFO("Estimation torque on joins: %f, %f, %f, %f, %f, %f, %f\n", grav_torques(0), grav_torques(1),grav_torques(2), grav_torques(3), grav_torques(4), grav_torques(5), grav_torques(6));
      for(int i = 0; i < 7; i++){
          u(i) = KP[i]*(q_dest(i)-q(i)) - KD[i]*dq(i) + grav_torques(i);
          torq_msg.data = u(i);
          torque_controller_pub.at(i).publish(torq_msg);
      }
      counter++;
      ros::spinOnce();
      loop_rate.sleep();
    }
    return 0;
}

void iiwaStateCallback(const sensor_msgs::JointState msg)
{
    for (int i=0; i< msg.name.size(); i++){
        q(i) = msg.position[i];
        dq(i) = msg.velocity[i];
        ddq(i) = 0.0;
        meas_torques(i) = msg.effort[i];
    }
}

void startControllers(ros::NodeHandle & nh){
  // Start torque controllers. If you want to use position controllers, change 'joint1_torque_controller' to 'joint1_position_controller' and and so on.
  ros::service::waitForService("/iiwa/controller_manager/switch_controller");
  ros::ServiceClient switch_controller = nh.serviceClient<controller_manager_msgs::SwitchController>("/iiwa/controller_manager/switch_controller");
  controller_manager_msgs::SwitchController srv;
  srv.request.start_controllers = {"joint1_torque_controller", "joint2_torque_controller", "joint3_torque_controller",
                                   "joint4_torque_controller", "joint5_torque_controller", "joint6_torque_controller",
                                   "joint7_torque_controller"};
  srv.request.strictness = srv.request.STRICT;
  bool call_success = switch_controller.call(srv);
}
