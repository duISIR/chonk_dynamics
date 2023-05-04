
#ifndef __se3_centroidal_orientation_hpp__
#define __se3_centroidal_orientation_hpp__

// Eigen headers
#include <Eigen/Core>
#include <Eigen/Dense>
// iDynTree headers
#include <iDynTree/Model/FreeFloatingState.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/ModelIO/ModelLoader.h>

// Helpers function to convert between
// iDynTree datastructures
#include <iDynTree/Core/EigenHelpers.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/WrenchStamped.h>
#include "X_transformation.hpp"
#include "i_localgenerate.hpp"
#include "I_spatial_generation.hpp"

class EVA
{
  public:

    // declare joint subscriber
    ros::Subscriber joint_sub;
    ros::Subscriber joint_sub_base;
    ros::Subscriber sensor_sub_right;
    ros::Subscriber sensor_sub_left;
    ros::Subscriber joint_acc_sub;
    // declare inertia publishers
    ros::Publisher sensor_pub_right;
    ros::Publisher sensor_pub_left;
//    ros::Publisher acc_box_pub;
    // define messages
    std_msgs::Float64MultiArray msg_actual_right;
    std_msgs::Float64MultiArray msg_actual_left;
//    std_msgs::Float64MultiArray msg_acc_box;
    // define the urdf file used by idyntree, this urdf is generated from gazebo
  //  std::string URDF_FILE = "/home/dwq/chonk/src/chonk_pushing/urdf/chonk_gazebo_fixed_wheels_combinebody.urdf";
    std::string URDF_FILE;
    // load urdf model
    iDynTree::KinDynComputations kinDynComp;
    iDynTree::ModelLoader mdlLoader;
    // define actual joint configuration and velocity feedbacks
    Eigen::VectorXd q_feedback_gazebo;
    Eigen::VectorXd dq_feedback_gazebo;
    // The gravity acceleration is a 3d acceleration vector.
    Eigen::Vector3d gravity;
    // declare floating-base and joint configuration and velocity variables
    iDynTree::Vector4 quaternion;
    Eigen::Matrix3d rotation_base;
    Eigen::Vector3d p_base;
    Eigen::Matrix4d world_T_base;
    Eigen::VectorXd s;
    Eigen::VectorXd ds;
    Eigen::VectorXd dds;
    Eigen::Vector3d base_linear_velocity_global;
    Eigen::Vector3d base_angular_velocity_local;
    Eigen::VectorXd base_velocity;
    Eigen::VectorXd base_acceleration;
    // declare jacobian variables to achieve the right arm ee grasp Jacobian
  //  Eigen::MatrixXd Jac_JOINT5_right(6, 6 + Dofs);   Jac_JOINT5_right.setZero();
    Eigen::MatrixXd Jac_ee_right;
  //  Eigen::MatrixXd G_T_JOINT5_right(4, 4);          G_T_JOINT5_right.setZero();
    // declare jacobian variables to achieve the left arm ee grasp Jacobian
  //  Eigen::MatrixXd Jac_JOINT5_left(6, 6 + Dofs);    Jac_JOINT5_left.setZero();
    Eigen::MatrixXd Jac_ee_left;
  //  Eigen::MatrixXd G_T_JOINT5_left(4, 4);           G_T_JOINT5_left.setZero();
    // declare ee and ft position relationship variables
  //  Eigen::Vector3d JOINT5_p_ee_right;       JOINT5_p_ee_right << -0.08, 0, -0.039-0.04-0.0333;
  //  Eigen::Vector3d JOINT5_p_ee_left;        JOINT5_p_ee_left << -0.08, 0, -0.039-0.04-0.0333;
  //  Eigen::MatrixXd X_transform_right(6,6);  X_transform_right.setIdentity(6,6);
  //  Eigen::MatrixXd X_transform_left(6,6);   X_transform_left.setIdentity(6,6);
    // declare variables of mass matrix M, centripedal and coriolis force C and gravity force G
    Eigen::MatrixXd M;
    Eigen::MatrixXd M_inv;
    Eigen::VectorXd C_G;
    Eigen::VectorXd C;
    Eigen::VectorXd G;
//    iDynTree::FreeFloatingGeneralizedTorques C_G(kinDynComp.model());
//    iDynTree::FreeFloatingGeneralizedTorques G(kinDynComp.model());
    int Dofs;
    int link_num;
    bool ok;
    // declare transform variables between sensor frame and two ee frames
    Eigen::Matrix4d sensor_H_ee_r;
    Eigen::Matrix4d sensor_H_ee_l;
    // declare spatial transform variables between sensor frame and two ee frames
    Eigen::MatrixXd sensor_X_ee_r;
    Eigen::MatrixXd sensor_X_ee_l;
    // declare transform variables of two ee frames expressed in global frame
    Eigen::Matrix4d G_H_ee_r;
    Eigen::Matrix4d G_H_ee_l;
    // declare transform variables of two ee frames expressed in global frame
    Eigen::MatrixXd G_X_ee_r;
    Eigen::MatrixXd G_X_ee_l;
    // declare operational-space inertia variables
    Eigen::MatrixXd G_lambda_r;
    Eigen::MatrixXd G_lambda_l;
    // declare operational-space consistent inverse of Jacobian variables with transpose
    Eigen::MatrixXd G_J_ee_bar_r_T;
    Eigen::MatrixXd G_J_ee_bar_l_T;
    // declare operational-space bias variables
    Eigen::VectorXd G_bias_r;
    Eigen::VectorXd G_bias_l;
    // declare operational-space colios variables
    Eigen::VectorXd G_mu_r;
    Eigen::VectorXd G_mu_l;
    // declare operational-space gravity variables
    Eigen::VectorXd G_rho_r;
    Eigen::VectorXd G_rho_l;
    // declare operational-space acceleration variables
    Eigen::VectorXd G_acc_ee_r;
    Eigen::VectorXd G_acc_ee_l;
    // declare operational-space force variables
    Eigen::VectorXd G_F_ee_r;
    Eigen::VectorXd G_F_ee_l;
    // declare subscribed sensors
    Eigen::VectorXd ft_sensor_r;
    Eigen::VectorXd ft_sensor_l;
    // declare derivation between real sensor values and operational-space force
    Eigen::VectorXd ft_ee_r_actual;
    Eigen::VectorXd ft_ee_l_actual;


    int i_callback;

    Eigen::MatrixXd sensor_I_ee_r_conventional;
    Eigen::MatrixXd I_ee_r_conventional;
    float m_ee_r;
    Eigen::VectorXd Gravity_ee_r_conventional;
    Eigen::Vector3d sensor_p_ee_r;
    Eigen::Matrix3d sensor_I_angular_ee_r;
    Eigen::MatrixXd G_I_ee_r_conventional;
    Eigen::VectorXd G_v_ee_r_conventional;
    Eigen::MatrixXd sensor_X_ee_r_conventional;
    Eigen::MatrixXd G_X_ee_r_conventional;
    Eigen::VectorXd G_acc_r_conventional;
    Eigen::VectorXd G_F_r_conventional;


    Eigen::MatrixXd sensor_I_ee_l_conventional;
    Eigen::MatrixXd I_ee_l_conventional;
    float m_ee_l;
    Eigen::VectorXd Gravity_ee_l_conventional;
    Eigen::Vector3d sensor_p_ee_l;
    Eigen::Matrix3d sensor_I_angular_ee_l;
    Eigen::MatrixXd G_I_ee_l_conventional;
    Eigen::VectorXd G_v_ee_l_conventional;
    Eigen::MatrixXd sensor_X_ee_l_conventional;
    Eigen::MatrixXd G_X_ee_l_conventional;
    Eigen::VectorXd G_acc_l_conventional;
    Eigen::VectorXd G_F_l_conventional;


    EVA(ros::NodeHandle & nh){
      // declare joint subscriber
      joint_sub = nh.subscribe("/chonk/joint_states", 1, &EVA::read_joint_states_cb, this);
      joint_sub_base = nh.subscribe("/chonk/base_pose_ground_truth", 1, &EVA::read_base_states_cb, this);
      sensor_sub_right = nh.subscribe("/ft_right/raw/data", 1, &EVA::read_ft_sensor_right_data_cb, this);
      sensor_sub_left = nh.subscribe("/ft_left/raw/data", 1, &EVA::read_ft_sensor_left_data_cb, this);
      joint_acc_sub = nh.subscribe("/chonk/joint_acc_pub", 1, &EVA::read_joint_acc_cb, this);

      // declare inertia publishers
      sensor_pub_right = nh.advertise<std_msgs::Float64MultiArray>("/chonk/sensor_ft_right", 10);
      sensor_pub_left = nh.advertise<std_msgs::Float64MultiArray>("/chonk/sensor_ft_left", 10);

//      acc_box_pub = nh.advertise<std_msgs::Float64MultiArray>("/chonk/acc_box", 10);
      // load model
      URDF_FILE = "/home/dwq/chonk/src/chonk_pushing/urdf/chonk_pushing.urdf";
      ok = mdlLoader.loadModelFromFile(URDF_FILE);
      ok = kinDynComp.loadRobotModel(mdlLoader.model());
      // set the frame reference, all expressed in inertial fixed frame
      kinDynComp.setFrameVelocityRepresentation(iDynTree::INERTIAL_FIXED_REPRESENTATION);
      // print model informations
      std::cout << "The loaded model has " << kinDynComp.getNrOfDegreesOfFreedom()
                << " internal DOFs and " << kinDynComp.getNrOfLinks() << " links." << std::endl;
      std::cout << "The floating base is " << kinDynComp.getFloatingBase() << std::endl;
      // get number of DOFs and Links
      Dofs = kinDynComp.getNrOfDegreesOfFreedom();
      link_num = kinDynComp.getNrOfLinks();

      q_feedback_gazebo.setZero(Dofs);
      dq_feedback_gazebo.setZero(Dofs);
      gravity << 0.0, 0.0, -9.8;
      rotation_base.setIdentity();
      p_base.setZero();
      world_T_base << rotation_base,                       p_base,
                      Eigen::Vector3d::Zero().transpose(),      1;
      s.setZero(Dofs);
      base_linear_velocity_global.setZero(3);
      base_angular_velocity_local.setZero(3);
      base_velocity.setZero(6);
      base_acceleration.setZero(6);
      ds.setZero(Dofs);
      dds.setZero(Dofs);
      ok = kinDynComp.getRelativeTransform("RARM_changer_link_base", "RARM_END_EFFECTOR_grasp", sensor_H_ee_r);
      ok = kinDynComp.getRelativeTransform("LARM_changer_link_base", "LARM_END_EFFECTOR_grasp", sensor_H_ee_l);
      sensor_X_ee_r.setZero(6,6);
      sensor_X_ee_l.setZero(6,6);
      sensor_X_ee_r = se3::parent_X_local(sensor_H_ee_r.topRightCorner(3,1), sensor_H_ee_r.topLeftCorner(3,3));
      sensor_X_ee_l = se3::parent_X_local(sensor_H_ee_l.topRightCorner(3,1), sensor_H_ee_l.topLeftCorner(3,3));



      m_ee_r = 0.3113;
      sensor_I_ee_r_conventional.resize(6,6);
      float ixx=0.00064665, ixy=0, ixz=0.000297068, iyy=0.00082646, iyz=0, izz=0.000354023;
      sensor_p_ee_r << -0.01773,  0,  0.04772;
      sensor_I_angular_ee_r << se3::i_localgenerate(ixx, ixy, ixz, iyy, iyz, izz);
      sensor_I_ee_r_conventional = se3::I_spatial_generate(m_ee_r, sensor_p_ee_r, sensor_I_angular_ee_r); // Notice: This spatial inertia is based on Roy Featherstone.
      sensor_X_ee_r_conventional.setZero(6,6);
      sensor_X_ee_r_conventional = se3::parent_X_local_conventional(sensor_H_ee_r.topRightCorner(3,1), sensor_H_ee_r.topLeftCorner(3,3));
      I_ee_r_conventional.setZero(6,6);
      I_ee_r_conventional = sensor_X_ee_r_conventional.transpose() * sensor_I_ee_r_conventional * sensor_X_ee_r_conventional;
      G_I_ee_r_conventional.setZero(6,6);
      G_v_ee_r_conventional.setZero(6);
      G_X_ee_r_conventional.setZero(6,6);
      G_acc_r_conventional.setZero(6);
      Gravity_ee_r_conventional.setZero(6);
      Gravity_ee_r_conventional[5] = m_ee_r * gravity[2];
      G_F_r_conventional.setZero(6);


      m_ee_l = 0.3113;
      sensor_I_ee_l_conventional.resize(6,6);
      ixx=0.00064665, ixy=0, ixz=0.000297068, iyy=0.00082646, iyz=0, izz=0.000354023;
      sensor_p_ee_l << -0.01773,  0,   0.04772;
      sensor_I_angular_ee_l << se3::i_localgenerate(ixx, ixy, ixz, iyy, iyz, izz);
      sensor_I_ee_l_conventional = se3::I_spatial_generate(m_ee_l, sensor_p_ee_l, sensor_I_angular_ee_l); // Notice: This spatial inertia is based on Roy Featherstone.
      sensor_X_ee_l_conventional.setZero(6,6);
      sensor_X_ee_l_conventional = se3::parent_X_local_conventional(sensor_H_ee_l.topRightCorner(3,1), sensor_H_ee_l.topLeftCorner(3,3));
      I_ee_l_conventional.setZero(6,6);
      I_ee_l_conventional = sensor_X_ee_l_conventional.transpose() * sensor_I_ee_l_conventional * sensor_X_ee_l_conventional;
      G_I_ee_l_conventional.setZero(6,6);
      G_v_ee_l_conventional.setZero(6);
      G_X_ee_l_conventional.setZero(6,6);
      G_acc_l_conventional.setZero(6);
      Gravity_ee_l_conventional.setZero(6);
      Gravity_ee_l_conventional[5] = m_ee_l * gravity[2];
      G_F_l_conventional.setZero(6);


      G_H_ee_r.setZero(4,4);
      G_H_ee_l.setZero(4,4);
      G_X_ee_r.setZero(6,6);
      G_X_ee_l.setZero(6,6);

      G_lambda_r.setZero(6, 6);
      G_lambda_l.setZero(6, 6);
      G_bias_r.setZero(6);
      G_bias_l.setZero(6);
      G_mu_r.setZero(6);
      G_mu_l.setZero(6);
      G_rho_r.setZero(6);
      G_rho_l.setZero(6);
      M.setZero(6+Dofs, 6+Dofs);
      M_inv.setZero(6+Dofs, 6+Dofs);
      C_G.setZero(6+Dofs);
      C.setZero(6+Dofs);
      G.setZero(6+Dofs);
      Jac_ee_right.setZero(6, 6+Dofs);
      Jac_ee_left.setZero(6, 6+Dofs);
      G_acc_ee_r.setZero(6);
      G_acc_ee_l.setZero(6);
      G_J_ee_bar_r_T.setZero(6, 6+Dofs);
      G_J_ee_bar_l_T.setZero(6, 6+Dofs);
      G_F_ee_r.setZero(6);
      G_F_ee_l.setZero(6);

      ft_sensor_r.setZero(6);
      ft_sensor_l.setZero(6);
      ft_ee_r_actual.setZero(6);
      ft_ee_l_actual.setZero(6);

//      ok = kinDynComp.getFrameFreeFloatingJacobian("RARM_END_EFFECTOR_grasp_end", Jac_ee_right);
//      ok = kinDynComp.getFrameFreeFloatingJacobian("LARM_END_EFFECTOR_grasp_end", Jac_ee_left);

      msg_actual_right.layout.dim.push_back(std_msgs::MultiArrayDimension());
      msg_actual_right.layout.dim[0].size = 6;
      msg_actual_right.layout.dim[0].stride = 1;
      msg_actual_right.layout.dim[0].label = "sensor_ft_right";
      msg_actual_right.data.resize(6);

      msg_actual_left.layout.dim.push_back(std_msgs::MultiArrayDimension());
      msg_actual_left.layout.dim[0].size = 6;
      msg_actual_left.layout.dim[0].stride = 1;
      msg_actual_left.layout.dim[0].label = "sensor_ft_left";
      msg_actual_left.data.resize(6);

//      msg_acc_box.layout.dim.push_back(std_msgs::MultiArrayDimension());
//      msg_acc_box.layout.dim[0].size = 6;
//      msg_acc_box.layout.dim[0].stride = 1;
//      msg_acc_box.layout.dim[0].label = "acc_box";
//      msg_acc_box.data.resize(6);

      std::cout << kinDynComp.getDescriptionOfDegreesOfFreedom() << std::endl;

//      std::cout << kinDynComp.model().getJointIndex("CHEST_JOINT0") << std::endl;
//      std::cout << kinDynComp.model().getJointIndex("HEAD_JOINT0") << std::endl;
//      std::cout << kinDynComp.model().getJointIndex("HEAD_JOINT1") << std::endl;
//      std::cout << kinDynComp.model().getJointIndex("RARM_JOINT0") << std::endl;
//      std::cout << kinDynComp.model().getJointIndex("RARM_JOINT1") << std::endl;
//      std::cout << kinDynComp.model().getJointIndex("RARM_JOINT2") << std::endl;
//      std::cout << kinDynComp.model().getJointIndex("RARM_JOINT3") << std::endl;
//      std::cout << kinDynComp.model().getJointIndex("RARM_JOINT4") << std::endl;
//      std::cout << kinDynComp.model().getJointIndex("RARM_JOINT5") << std::endl;
//      std::cout << kinDynComp.model().getJointIndex("LARM_JOINT0") << std::endl;
//      std::cout << kinDynComp.model().getJointIndex("LARM_JOINT1") << std::endl;
//      std::cout << kinDynComp.model().getJointIndex("LARM_JOINT2") << std::endl;
//      std::cout << kinDynComp.model().getJointIndex("LARM_JOINT3") << std::endl;
//      std::cout << kinDynComp.model().getJointIndex("LARM_JOINT4") << std::endl;
//      std::cout << kinDynComp.model().getJointIndex("LARM_JOINT5") << std::endl;


    }
    ~EVA(){}

    void read_joint_states_cb(const sensor_msgs::JointState::ConstPtr& msg);
    void read_base_states_cb(const nav_msgs::Odometry::ConstPtr& msg);
    void read_ft_sensor_right_data_cb(const geometry_msgs::WrenchStamped::ConstPtr& msg);
    void read_ft_sensor_left_data_cb(const geometry_msgs::WrenchStamped::ConstPtr& msg);
    void read_joint_acc_cb(const std_msgs::Float64MultiArray::ConstPtr& msg);
};


#endif
