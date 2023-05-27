#include "EVA.hpp"

// joint motion callback
void EVA::read_joint_states_cb(const sensor_msgs::JointState::ConstPtr& msg)
{
  // DOF Index: 0 Name: rl_wheel_joint
  s[0] = msg->position[19];           ds[0]  = msg->velocity[19];
  // DOF Index: 1 Name: fl_wheel_joint
  s[1] = msg->position[15];           ds[1]  = msg->velocity[15];
  // DOF Index: 2 Name: rr_wheel_joint
  s[2] = msg->position[21];           ds[2]  = msg->velocity[21];
  // DOF Index: 3 Name: fr_wheel_joint
  s[3] = msg->position[17];           ds[3]  = msg->velocity[17];
  // DOF Index: 4 Name: fr_wheel_link_passive_joint
  s[4] = msg->position[18];           ds[4]  = msg->velocity[18];
  // DOF Index: 5 Name: rr_wheel_link_passive_joint
  s[5] = msg->position[22];           ds[5]  = msg->velocity[22];
  // DOF Index: 6 Name: fl_wheel_link_passive_joint
  s[6] = msg->position[16];           ds[6]  = msg->velocity[16];
  // DOF Index: 7 Name: rl_wheel_link_passive_joint
  s[7] = msg->position[20];           ds[7]  = msg->velocity[20];
  // DOF Index: 8 Name: CHEST_JOINT0
  s[8] = msg->position[0];            ds[8]  = msg->velocity[0];
  // DOF Index: 9 Name: HEAD_JOINT0
  s[9] = msg->position[1];            ds[9]  = msg->velocity[1];
  // DOF Index: 10 Name: RARM_JOINT0
  s[10] = msg->position[9];           ds[10] = msg->velocity[9];
  // DOF Index: 11 Name: LARM_JOINT0
  s[11] = msg->position[3];           ds[11] = msg->velocity[3];
  // DOF Index: 12 Name: LARM_JOINT1
  s[12] = msg->position[4];           ds[12] = msg->velocity[4];
  // DOF Index: 13 Name: LARM_JOINT2
  s[13] = msg->position[5];           ds[13] = msg->velocity[5];
  // DOF Index: 14 Name: LARM_JOINT3
  s[14] = msg->position[6];           ds[14] = msg->velocity[6];
  // DOF Index: 15 Name: LARM_JOINT4
  s[15] = msg->position[7];           ds[15] = msg->velocity[7];
  // DOF Index: 16 Name: LARM_JOINT5
  s[16] = msg->position[8];           ds[16] = msg->velocity[8];
  // DOF Index: 17 Name: RARM_JOINT1
  s[17] = msg->position[10];          ds[17] = msg->velocity[10];
  // DOF Index: 18 Name: RARM_JOINT2
  s[18] = msg->position[11];          ds[18] = msg->velocity[11];
  // DOF Index: 19 Name: RARM_JOINT3
  s[19] = msg->position[12];          ds[19] = msg->velocity[12];
  // DOF Index: 20 Name: RARM_JOINT4
  s[20] = msg->position[13];          ds[20] = msg->velocity[13];
  // DOF Index: 21 Name: RARM_JOINT5
  s[21] = msg->position[14];          ds[21] = msg->velocity[14];
  // DOF Index: 22 Name: HEAD_JOINT1
  s[22] = msg->position[2];           ds[22] = msg->velocity[2];
}

// donkey motion callback
void EVA::read_base_states_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
  p_base << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;

  quaternion[0] = msg->pose.pose.orientation.w;
  quaternion[1] = msg->pose.pose.orientation.x;
  quaternion[2] = msg->pose.pose.orientation.y;
  quaternion[3] = msg->pose.pose.orientation.z;
  rotation_base = iDynTree::toEigen(iDynTree::Rotation::RotationFromQuaternion(quaternion));

  world_T_base << rotation_base,                       p_base,
                  Eigen::Vector3d::Zero().transpose(),      1;

  base_linear_velocity_global << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z;
  base_angular_velocity_local << msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z;

//  base_velocity << base_linear_velocity_global, rotation_base * base_angular_velocity_local;
  base_velocity << base_linear_velocity_global, base_angular_velocity_local;

}

// right sensor callback
void EVA::read_ft_sensor_right_data_cb(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
  ft_sensor_r << msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z, msg->wrench.force.x, msg->wrench.force.y-1.9, msg->wrench.force.z;
}

// left sensor callback
void EVA::read_ft_sensor_left_data_cb(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
  ft_sensor_l << msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z, msg->wrench.force.x, msg->wrench.force.y+2.16, msg->wrench.force.z;
}

// right ee acceleration callback
void EVA::read_joint_acc_cb(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  //Base acceleration
  base_acceleration << msg->data[0], msg->data[1], 0, 0, 0, msg->data[2];
  // DOF Index: 8 Name: CHEST_JOINT0
  dds[8] = msg->data[3];
  // DOF Index: 9 Name: HEAD_JOINT0
  dds[9] = msg->data[4];
  // DOF Index: 10 Name: RARM_JOINT0
  dds[10] = msg->data[12];
  // DOF Index: 11 Name: LARM_JOINT0
  dds[11] = msg->data[6];
  // DOF Index: 12 Name: LARM_JOINT1
  dds[12] = msg->data[7];
  // DOF Index: 13 Name: LARM_JOINT2
  dds[13] = msg->data[8];
  // DOF Index: 14 Name: LARM_JOINT3
  dds[14] = msg->data[9];
  // DOF Index: 15 Name: LARM_JOINT4
  dds[15] = msg->data[10];
  // DOF Index: 16 Name: LARM_JOINT5
  dds[16] = msg->data[11];
  // DOF Index: 17 Name: RARM_JOINT1
  dds[17] = msg->data[13];
  // DOF Index: 18 Name: RARM_JOINT2
  dds[18] = msg->data[14];
  // DOF Index: 19 Name: RARM_JOINT3
  dds[19] = msg->data[15];
  // DOF Index: 20 Name: RARM_JOINT4
  dds[20] = msg->data[16];
  // DOF Index: 21 Name: RARM_JOINT5
  dds[21] = msg->data[17];
  // DOF Index: 22 Name: HEAD_JOINT1
  dds[22] = msg->data[5];
}







