#include "piperx_sim_control/piperx_sim_control.hpp"


PiperXSimControl::PiperXSimControl() : Node("piperx_sim_control")
{
  RCLCPP_INFO(this->get_logger(), "Control node has started......");

  current_state_ = PickState::OPEN_GRIPPER;

  has_cube_pose_ = false;

  required_cube_samples_ = 1;
  cube_sample_count_ = 0;

  cube_sum_x_ = 0.0;
  cube_sum_y_ = 0.0;
  cube_sum_z_ = 0.0;

  // Tuned for grasping the simulated cube (5 cm sides).
  gripper_grasp_joints_ = {0.015, -0.015};

  cube_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/aruco/cube_pose_base",
    1,
    std::bind(&PiperXSimControl::cubePoseCallback, this, std::placeholders::_1)
  );

}

void PiperXSimControl::cubePoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (current_state_ != PickState::WAIT_FOR_MARKER)
  {
    return;
  }

  if (has_cube_pose_)
  {
    return;
  }

  cube_sum_x_ += msg->pose.position.x;
  cube_sum_y_ += msg->pose.position.y;
  cube_sum_z_ += msg->pose.position.z;

  cube_sample_count_++;

  RCLCPP_INFO(
    this->get_logger(),
    "Collected marker sample %d/%d: x=%.3f, y=%.3f, z=%.3f",
    cube_sample_count_,
    required_cube_samples_,
    msg->pose.position.x,
    msg->pose.position.y,
    msg->pose.position.z
  );

  if (cube_sample_count_ == required_cube_samples_)
  {
    cube_pose_ = *msg;

    cube_pose_.pose.position.x = cube_sum_x_ / cube_sample_count_;
    cube_pose_.pose.position.y = cube_sum_y_ / cube_sample_count_;
    cube_pose_.pose.position.z = cube_sum_z_ / cube_sample_count_;

    has_cube_pose_ = true;
  }
}

void PiperXSimControl::initializeMoveIt()
{
  arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    shared_from_this(), "arm");

  gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    shared_from_this(), "gripper");

  arm_group_->setMaxVelocityScalingFactor(1);
  arm_group_->setMaxAccelerationScalingFactor(1);

  if (!arm_group_->setEndEffectorLink("gripper_tcp"))
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to set gripper_tcp as end-effector link.");
  }
}  

void PiperXSimControl::runStateMachine()
{
  switch (current_state_)
  {
    case PickState::OPEN_GRIPPER:
      RCLCPP_INFO(this->get_logger(), "State: OPEN_GRIPPER");

      moveGripperJoints(gripper_open_joints_);

      current_state_ = PickState::MOVE_TO_SCAN;

      break; 

    case PickState::MOVE_TO_SCAN:
      RCLCPP_INFO(this->get_logger(), "State: MOVE_TO_SCAN");

      if (moveArmJoints(scan_pose_joints_))
      {
        current_state_ = PickState::WAIT_FOR_MARKER;
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Scan pose failed, retrying...");
      } 

      break;
    
    case PickState::WAIT_FOR_MARKER:

      if (has_cube_pose_)
      {
        RCLCPP_INFO(
          this->get_logger(),
          "Target marker pose: x=%.3f, y=%.3f, z=%.3f",
          cube_pose_.pose.position.x,
          cube_pose_.pose.position.y,
          cube_pose_.pose.position.z
        );

        current_state_ = PickState::MOVE_TO_PICK;
      }
  
      break;
    
    case PickState::MOVE_TO_PICK:
      RCLCPP_INFO(this->get_logger(), "State: MOVE_TO_PICK");

      if (moveTcpToCube())
      {
        current_state_ = PickState::GRASP;
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Pick failed, retrying...");
      }

      break;

    case PickState::GRASP:
      RCLCPP_INFO(this->get_logger(), "State: GRASP");

      moveGripperJoints(gripper_grasp_joints_);

      current_state_ = PickState::PLACE;

      break;

    case PickState::PLACE:

      // to be done

      break;

    case PickState::DONE:
        
      break;

    default:
      RCLCPP_INFO(this->get_logger(), "State not implemented yet.");

      break;
  }
}

bool PiperXSimControl::moveTcpToCube()
{
  geometry_msgs::msg::Pose target_pose;

  target_pose.position.x = cube_pose_.pose.position.x;
  target_pose.position.y = cube_pose_.pose.position.y;
  target_pose.position.z = cube_pose_.pose.position.z;

  target_pose.orientation.x = 0.0;
  target_pose.orientation.y = 1.0;
  target_pose.orientation.z = 0.0;
  target_pose.orientation.w = 0.0;

  arm_group_->setStartStateToCurrentState();
  arm_group_->setPoseTarget(target_pose);

  if (arm_group_->plan(arm_plan_) == moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_INFO(this->get_logger(), "Pregrasp plan succeeded. Executing...");
    auto result = arm_group_->execute(arm_plan_);
    arm_group_->clearPoseTargets();

    if (result == moveit::core::MoveItErrorCode::SUCCESS)
    {
      return true;
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Execute failed.");
      return false;
    }
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Pregrasp plan failed.");
    arm_group_->clearPoseTargets();
    return false;
  }
}

bool PiperXSimControl::moveArmJoints(const std::vector<double> & joint_angles)
{
  arm_group_->setJointValueTarget(joint_angles);

  if (arm_group_->plan(arm_plan_) == moveit::core::MoveItErrorCode::SUCCESS)
  {
    auto result = arm_group_->execute(arm_plan_);

    if (result == moveit::core::MoveItErrorCode::SUCCESS)
    {
      return true;
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Arm execute failed.");
      return false;
    }
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Arm plan failed.");
    return false;
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
    RCLCPP_ERROR(this->get_logger(), "Gripper plan failed");
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<PiperXSimControl>();

  node->initializeMoveIt();

  rclcpp::Rate rate = rclcpp::Rate(30);

  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);

    node->runStateMachine();

    rate.sleep();
  }

  rclcpp::shutdown();

  return 0;
}