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
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    try
    {
        geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform("ee_frame_", "tracked_frame_", tf2::TimePointZero);

        Eigen::Vector3d translation(
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        );

        Eigen::Quaterniond rotation(
            transform.transform.rotation.w,
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z
        )

        translational_errors_[circle_idx_] = computeTranslationMetric(translation);
        rotational_errors_[circle_idx_] = computeRotationMetric(rotation);

        RCLCPP::INFO(
            this->get_logger(),
            "Point %zu | Translation error: %.6f m | Rotation error: %.6f deg",
            circle_idx_,
            translational_errors_[circle_idx_];
            rotational_errors_[circle_idx_];
        );
    }
    catch (const tf2::TransformException & ex)
    {
        RCLCPP_ERROR(this->get_logger(), "lookup failed!");
    }
}

double RobotKinValidation::computeRotationMetric(const Eigen::Quaterniond & rot)
{
    Eigen::Quaterniond rot_norm = rot.normalized();
    Eigen::AngleAxisd ax_ang(rot_norm);
    return ax_ang.angle() * 180.0 / M_PI;

}


double RobotKinValidation::computeTranslationMetric(Eigen::Vector3d & pos_vec)
{
    return pos_vec.norm();
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