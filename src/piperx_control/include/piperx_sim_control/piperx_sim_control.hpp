#ifndef PIPERX_SIM_CONTROL__PIPERX_SIM_CONTROL_HPP_
#define PIPERX_SIM_CONTROL__PIPERX_SIM_CONTROL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

class PiperXSimControl : public rclcpp::Node
{
public:

  PiperXSimControl();

  ~PiperXSimControl() = default;

  void mainJointMovement();

  void moveArmJoints(const std::vector<double> & joint_angles);

  void moveGripperJoints(const std::vector<double> & joint_angles);


private:

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;

  moveit::planning_interface::MoveGroupInterface::Plan arm_plan_;

  moveit::planning_interface::MoveGroupInterface::Plan gripper_plan_;

  std::vector<double> scan_pose_joints_ = {0.0, 0.373, -1.283, 1.315, 0.0, 0.0};

  std::vector<double> gripper_open_joints_ = {0.050, -0.050};
};

#endif 