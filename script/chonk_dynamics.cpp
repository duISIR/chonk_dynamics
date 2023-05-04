#include <ros/ros.h>
#include <cstdlib>
#include "signal.h"
#include <time.h>
#include <stdio.h>
#include "pseudoinverse.hpp"
#include "skew.hpp"
#include "EVA.hpp"
#include <fstream>

void mySigintHandler(int sig)
{ ROS_INFO("Stop Towr Motion Generator"); ros::shutdown(); }


int main(int argc, char **argv)
{
  ros::init(argc, argv, "chonk_dynamics", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;
  signal(SIGINT, mySigintHandler);
  ROS_INFO("Start chonk dynamics calculation");
  /* Define the publish rate. */
  ros::Rate loop_rate(100); // Run the publishers in fixed rate.
  EVA eva(nh);
  bool ok;
  Eigen::VectorXd v_temp(6);
  Eigen::VectorXd a_box_temp(6);
  Eigen::VectorXd a_temp_right(6);
  Eigen::VectorXd a_temp_left(6);
  Eigen::VectorXd f_temp(6);
  Eigen::MatrixXd skew_motion_force(6,6);

  /*####################################################################################################################*/
  std::ofstream outputfiledwq;
  outputfiledwq.open("/home/dwq/chonk/src/plotting/script/ft.txt", std::ofstream::trunc | std::ofstream::out);
  /*####################################################################################################################*/

 
  while (ros::ok())
  {
    float current_time = ros::Time::now().toSec();

    // set all robot states
    eva.kinDynComp.setRobotState(iDynTree::make_matrix_view(eva.world_T_base),
                                 iDynTree::make_span(eva.s),
                                 iDynTree::make_span(eva.base_velocity),
                                 iDynTree::make_span(eva.ds),
                                 iDynTree::make_span(eva.gravity));
#if 0
    // calculate two right arm end-effector grasp Jacobian
    ok = eva.kinDynComp.getFrameFreeFloatingJacobian("RARM_END_EFFECTOR_grasp_end", eva.Jac_ee_right);
//    G_T_JOINT5_right = iDynTree::toEigen(kinDynComp.getWorldTransform("RARM_JOINT5_Link").asHomogeneousTransform());
//    X_transform_right.topRightCorner<3,3>() = -se3::skew(G_T_JOINT5_right.topLeftCorner<3,3>() * JOINT5_p_ee_right);
//    Jac_ee_right = X_transform_right * Jac_JOINT5_right;
    // calculate two right arm end-effector grasp Jacobian
    ok = eva.kinDynComp.getFrameFreeFloatingJacobian("LARM_END_EFFECTOR_grasp_end", eva.Jac_ee_left);
//    G_T_JOINT5_left = iDynTree::toEigen(kinDynComp.getWorldTransform("LARM_JOINT5_Link").asHomogeneousTransform());
//    X_transform_left.topRightCorner<3,3>() = -se3::skew(G_T_JOINT5_left.topLeftCorner<3,3>() * JOINT5_p_ee_left);
//    Jac_ee_left = X_transform_left * Jac_JOINT5_left;
    // calculate the mass matrix
    eva.kinDynComp.getFreeFloatingMassMatrix(eva.M);
    // calculate the centripedal and coriolis force, C, and gravity force G. In this step, it means C+G
    ok = eva.kinDynComp.generalizedBiasForces(eva.C_G);
    // calculate the gravity force G. In this step, it means G
    ok = eva.kinDynComp.generalizedGravityForces(eva.G);
    eva.C = eva.C_G - eva.G;
    // calculate the inverse of the mass matrix
    eva.M_inv = se3::pinv(eva.M);
    // calculate the operational-space inertia of two arm end-effector grasp
    eva.G_lambda_r = se3::pinv(eva.Jac_ee_right * eva.M_inv * eva.Jac_ee_right.transpose());
    eva.G_lambda_l = se3::pinv(eva.Jac_ee_left * eva.M_inv * eva.Jac_ee_left.transpose());
    // calculate the operational-space bias force of two arm end-effector grasp
    ok = eva.kinDynComp.getFrameBiasAcc("RARM_END_EFFECTOR_grasp", eva.G_bias_r);
    ok = eva.kinDynComp.getFrameBiasAcc("LARM_END_EFFECTOR_grasp", eva.G_bias_l);
    // calculate the operational-space dynamically-consistent Jacobians of two arm end-effector grasp
    eva.G_J_ee_bar_r_T = eva.G_lambda_r * eva.Jac_ee_right * eva.M_inv;
    eva.G_J_ee_bar_l_T = eva.G_lambda_r * eva.Jac_ee_left * eva.M_inv;
    // calculate the operational-space gravity forces of two arm end-effector grasp
    eva.G_rho_r = eva.G_J_ee_bar_r_T * eva.G;
    eva.G_rho_l = eva.G_J_ee_bar_r_T * eva.G;
    // calculate the operational-space forces of two arm end-effector grasp
    eva.G_F_ee_r = eva.G_lambda_r * eva.G_acc_ee_r + eva.G_mu_r + eva.G_rho_r;
    eva.G_F_ee_l = eva.G_lambda_l * eva.G_acc_ee_l + eva.G_mu_l + eva.G_rho_l;
#endif
    // calculate the transform matrices of ee frames expressed in global frame
    ok = eva.kinDynComp.getWorldTransform("RARM_END_EFFECTOR_grasp", eva.G_H_ee_r);
    ok = eva.kinDynComp.getWorldTransform("LARM_END_EFFECTOR_grasp", eva.G_H_ee_l);
    eva.G_X_ee_r.topLeftCorner(3,3) = eva.G_H_ee_r.topLeftCorner(3,3); eva.G_X_ee_r.bottomRightCorner(3,3) = eva.G_H_ee_r.topLeftCorner(3,3);
    eva.G_X_ee_l.topLeftCorner(3,3) = eva.G_H_ee_l.topLeftCorner(3,3); eva.G_X_ee_l.bottomRightCorner(3,3) = eva.G_H_ee_l.topLeftCorner(3,3);
    // calculate the deirvation between real sensor values and calculated operational-space forces
//    eva.Delta_ft_ee_r = -eva.G_X_ee_r.inverse().transpose() * eva.sensor_X_ee_r.transpose() * eva.ft_sensor_r - eva.G_F_ee_r;
//    eva.Delta_ft_ee_l = -eva.G_X_ee_l.inverse().transpose() * eva.sensor_X_ee_l.transpose() * eva.ft_sensor_l - eva.G_F_ee_l;

    // calculate the ee velocities of two arms
    ok = eva.kinDynComp.getFrameVel("RARM_END_EFFECTOR_grasp", v_temp);
    eva.G_v_ee_r_conventional << v_temp.tail(3), v_temp.head(3);
    eva.G_X_ee_r_conventional = eva.G_X_ee_r;
    eva.G_I_ee_r_conventional = eva.G_X_ee_r_conventional.inverse().transpose() * eva.I_ee_r_conventional * eva.G_X_ee_r_conventional.inverse();
    eva.G_acc_r_conventional << eva.G_acc_ee_r.tail(3), eva.G_acc_ee_r.head(3);
    skew_motion_force << se3::skew(eva.G_v_ee_r_conventional.head(3)),     se3::skew(eva.G_v_ee_r_conventional.tail(3)),
                         Eigen::Matrix3d::Zero(),                          se3::skew(eva.G_v_ee_r_conventional.head(3));
    ok = eva.kinDynComp.getFrameAcc("RARM_END_EFFECTOR_grasp", eva.base_acceleration, eva.dds, eva.G_acc_r_conventional);
    a_temp_right << eva.G_acc_r_conventional.tail(3), eva.G_acc_r_conventional.head(3);
    eva.G_F_r_conventional = eva.G_I_ee_r_conventional * a_temp_right +
                             skew_motion_force *  eva.G_I_ee_r_conventional * eva.G_v_ee_r_conventional +
                             eva.Gravity_ee_r_conventional;

    ok = eva.kinDynComp.getFrameVel("LARM_END_EFFECTOR_grasp", v_temp);
    eva.G_v_ee_l_conventional << v_temp.tail(3), v_temp.head(3);
    eva.G_X_ee_l_conventional = eva.G_X_ee_l;
    eva.G_I_ee_l_conventional = eva.G_X_ee_l_conventional.inverse().transpose() * eva.I_ee_l_conventional * eva.G_X_ee_l_conventional.inverse();
    eva.G_acc_l_conventional << eva.G_acc_ee_l.tail(3), eva.G_acc_ee_l.head(3);
    skew_motion_force << se3::skew(eva.G_v_ee_l_conventional.head(3)),     se3::skew(eva.G_v_ee_l_conventional.tail(3)),
                         Eigen::Matrix3d::Zero(),                          se3::skew(eva.G_v_ee_l_conventional.head(3));
    ok = eva.kinDynComp.getFrameAcc("LARM_END_EFFECTOR_grasp", eva.base_acceleration, eva.dds, eva.G_acc_l_conventional);
    a_temp_left << eva.G_acc_l_conventional.tail(3), eva.G_acc_l_conventional.head(3);
    eva.G_F_l_conventional = eva.G_I_ee_l_conventional * a_temp_left +
                             skew_motion_force *  eva.G_I_ee_l_conventional * eva.G_v_ee_l_conventional +
                             eva.Gravity_ee_l_conventional;

    // update message
//    a_box_temp = 0.5*(a_temp_right + a_temp_left);
//    eva.msg_acc_box.data = {a_box_temp[0], a_box_temp[1], a_box_temp[2], a_box_temp[3], a_box_temp[4], a_box_temp[5]};


//    std::cout << "force sensor result: " << std::endl;
//    std::cout << (eva.G_X_ee_r_conventional.inverse().transpose() * eva.sensor_X_ee_r_conventional.transpose() * eva.ft_sensor_r).transpose() << std::endl;
//    std::cout << "Inertial force/torque result: " << std::endl;
//    std::cout << eva.G_F_r_conventional.transpose() << std::endl;

//    std::cout << "force sensor result: " << std::endl;
//    std::cout << (eva.G_X_ee_l_conventional.inverse().transpose() * eva.sensor_X_ee_l_conventional.transpose() * eva.ft_sensor_l).transpose() << std::endl;
//    std::cout << "Inertial force/torque result: " << std::endl;
//    std::cout << eva.G_F_l_conventional.transpose() << std::endl;

    // calculate the deirvation between real sensor values and calculated operational-space forces
    eva.ft_ee_r_actual = -eva.G_X_ee_r_conventional.inverse().transpose() * eva.sensor_X_ee_r_conventional.transpose() * eva.ft_sensor_r + eva.G_F_r_conventional; //torque first and force second
    eva.ft_ee_l_actual = -eva.G_X_ee_l_conventional.inverse().transpose() * eva.sensor_X_ee_l_conventional.transpose() * eva.ft_sensor_l + eva.G_F_l_conventional; //torque first and force second

//    std::cout << "Actual right result: " << std::endl;
//    std::cout << eva.ft_ee_r_actual.transpose() << std::endl;
//    std::cout << "Actual left result: " << std::endl;
//    std::cout << eva.ft_ee_l_actual.transpose() << std::endl;

    // update message
    eva.msg_actual_right.data = {eva.ft_ee_r_actual[0], eva.ft_ee_r_actual[1], eva.ft_ee_r_actual[2], eva.ft_ee_r_actual[3], eva.ft_ee_r_actual[4], eva.ft_ee_r_actual[5]};
    eva.msg_actual_left.data = {eva.ft_ee_l_actual[0], eva.ft_ee_l_actual[1], eva.ft_ee_l_actual[2], eva.ft_ee_l_actual[3], eva.ft_ee_l_actual[4], eva.ft_ee_l_actual[5]};

    // publish message
    eva.sensor_pub_right.publish(eva.msg_actual_right);
    eva.sensor_pub_left.publish(eva.msg_actual_left);
//    eva.acc_box_pub.publish(eva.msg_acc_box);



//    outputfiledwq << ros::Time::now() << " "
//                  << eva.ft_ee_r_actual[3] << " "
//                  << eva.ft_ee_r_actual[4]<< " "
//                  << eva.ft_ee_r_actual[5]<< " "
//                  << eva.ft_ee_l_actual[3] << " "
//                  << eva.ft_ee_l_actual[4]<< " "
//                  << eva.ft_ee_l_actual[5]<< " "
//                  << std::endl;

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}


