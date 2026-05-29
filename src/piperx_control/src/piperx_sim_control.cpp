#include "piperx_sim_control/piperx_sim_control.hpp"

PiperXSimControl::PiperXSimControl() : Node("piperx_sim_control")
{
  RCLCPP_INFO(this->get_logger(), "Control node has started......");


  marker_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/aruco/marker_pose_base",
    10,
    std::bind(&PiperXSimControl::markerPoseCallback, this, std::placeholders::_1)
  );
}

void PiperXSimControl::markerPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  RCLCPP_INFO(
    this->get_logger(),
    "Marker pose in %s: x=%.3f, y=%.3f, z=%.3f",
    msg->header.frame_id.c_str(),
    msg->pose.position.x,
    msg->pose.position.y,
    msg->pose.position.z
  );
}

void PiperXSimControl::mainJointMovement()
{
  arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "arm");

  gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "gripper");

  moveArmJoints(scan_pose_joints_);

  moveGripperJoints(gripper_open_joints_);
}

void PiperXSimControl::moveArmJoints(const std::vector<double> & joint_angles)
{
  arm_group_->setJointValueTarget(joint_angles);

  if (arm_group_->plan(arm_plan_) == moveit::core::MoveItErrorCode::SUCCESS)
  {
    arm_group_->execute(arm_plan_);
  }
  else 
  {
    RCLCPP_INFO(this->get_logger(), "Arm plan failed");
  }
}

void PiperXSimControl::moveGripperJoints(const std::vector<double> & joint_angles)
{
  gripper_group_->setJointValueTarget(joint_angles);

  if (gripper_group_->plan(gripper_plan_) == moveit::core::MoveItErrorCode::SUCCESS)
  {
    gripper_group_->execute(gripper_plan_);
  }
  else 
  {
    RCLCPP_INFO(this->get_logger(), "Gripper plan failed");
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<PiperXSimControl>();

  node->mainJointMovement();

  rclcpp::spin(node);

  rclcpp::shutdown();
  
  return 0;
}