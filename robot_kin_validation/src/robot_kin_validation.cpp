#include <robot_kin_validation/robot_kin_validation.h>

RobotKinValidation::RobotKinValidation(rclcpp::NodeOptions options) : Node("robot_kin_validation", options)
{

}

void RobotKinValidation::initMoveGroup()
{
    // initialize movegroup
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "meca500_arm");

    // set movegroup to target joint angle
    move_group_->setJointValueTarget(target_joint_angs);

}

bool RobotKinValidation::mainCircleLoop()
{

}

void RobotKinValidation::solveIKAndMoveRobot(const Eigen::Vector3d & position)
{

}

bool RobotKinValidation::moveJoints()
{
    // if move plan succeeded, move robot
    if (move_group_->plan(plan_) == moveit::core::MoveItErrorCode::SUCCESS)
    {
        move_group_->execute(plan_);
    }
    else // print error otherwise
    {
        RCLCPP_INFO(this->get_logger(), "Plan failed");
    }

}

void RobotKinValidation::computeErrorMetrics()
{

}

double RobotKinValidation::computeRotationMetric(const Eigen::Quaterniond & rot)
{

}


double RobotKinValidation::computeTranslationMetric(Eigen::Vector3d & pos_vec)
{

}

void RobotKinValidation::robotDescriptionCallback(const std_msgs::msg::String& msg)
{

}

double RobotKinValidation::computeAverageOfArray(const std::array<double, 360> & array)
{

}

int main(int argc, char** argv)
{
    // initialize the node
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;

    options.automatically_declare_parameters_from_overrides(true);
    
    // create instance of class
    auto node = std::make_shared<RobotKinValidation>(options);
    rclcpp::sleep_for(std::chrono::seconds(2));
    
    node->initMoveGroup();

    while (rclcpp::ok())
    {
        if (node->mainCircleLoop())
            break;

        rclcpp::spin_some(node);
    }
    rclcpp::shutdown();
    return 0;
}