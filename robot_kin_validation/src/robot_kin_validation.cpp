#include <robot_kin_validation/robot_kin_validation.h>

RobotKinValidation::RobotKinValidation(rclcpp::NodeOptions options) 
    : Node("robot_kin_validation", options),
      circle_center_(0.23, 0.0, 0.22),
      circle_radius_(0.05),
      circle_idx_(0),
      ee_frame_("meca_axis_6_link"),
      tracked_frame_("")
{
    // initialize the error arrays to 0
    translational_errors_.fill(0.0);
    rotational_errors_.fill(0.0);

    // initialize the transform lookup objects
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // subscribe to robot description
    robot_description_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/robot_description",
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
        std::bind(&RobotKinValidation::robotDescriptionCallback, this, std::placeholders::_1)
    );
    
    // confirmation printouts
    RCLCPP_INFO(this->get_logger(), "RobotKinValidation node initialized.");
    RCLCPP_INFO(this->get_logger(), "Circle center: [%.3f, %.3f, %.3f], radius: %.3f m",
                circle_center_.x(), circle_center_.y(), circle_center_.z(), circle_radius_);

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
    if (circle_idx_ >= 360) {
        // computes the average translational and rotational errors
        double avg_trans = computeAverageOfArray(translational_errors_);
        double avg_rot = computeAverageOfArray(rotational_errors_);

        // worst translational error along the circle
        double max_trans = *std::max_element(translational_errors_.begin(),
                                             translational_errors_.end());

        // worst rotational error along the circle
        double max_rot = *std::max_element(rotational_errors_.begin(),
                                           rotational_errors_.end());

        // for reporting / debugging. Not necessary. 
        RCLCPP_INFO(this->get_logger(), "Circle complete.");
        RCLCPP_INFO(this->get_logger(), "Average translational error: %.6f m", avg_trans);
        RCLCPP_INFO(this->get_logger(), "Maximum translational error: %.6f m", max_trans);
        RCLCPP_INFO(this->get_logger(), "Average rotational error: %.6f deg", avg_rot);
        RCLCPP_INFO(this->get_logger(), "Maximum rotational error: %.6f deg", max_rot);

        return true;

    }
    double theta = static_cast<double>(circle_idx_) * M_PI / 180.0;

    // the circle remains on the yz plane
    Eigen::Vector3d target_position;
    target_position.x() = circle_center_.x();
    target_position.y() = circle_center_.y() + circle_radius_ * std::cos(theta);
    target_position.z() = circle_center_.z() + circle_radius_ * std::sin(theta);

    solveIKAndMoveRobot(target_position);
    circle_idx_++; 
    
    // circle not complete yet
    return false;
}

void RobotKinValidation::solveIKAndMoveRobot(const Eigen::Vector3d & position)
{
    if (ik_solver_ != nullptr) 
    {
        // setup an initial guess of the joint angles (check hint for RVIZ)
        KDL::JntArray q_init(chain_.getNrOfJoints());
        q_init(0) = M_PI/2.0; q_init(1) = 0.0; q_init(2) = 0.2; 
        q_init(3) = -M_PI/2.0; q_init(4) = 0.2; q_init(5) = 0.0; 

        // Convert Eigen::Vector3d to KDL::Vector
        KDL::Vector kdl_position(position(0), position(1), position(2));

        // Convert Eigen::Quaterniond to KDL::Rotation
        KDL::Rotation kdl_rotation = KDL::Rotation::Quaternion(
            0.0, 0.0, 0.0, 1.0  // meca500 EE orientation
        );

        // Construct the KDL::Frame
        KDL::Frame desired_ee_frame(kdl_rotation, kdl_position);

        // solve the inverse kinematics
        int error = ik_solver_->CartToJnt(q_init, desired_ee_frame, q_target_);

        if (error == KDL::ChainIkSolverPos_LMA::E_NOERROR)
        {
            // copy IK values into vector target_joint_angs
            target_joint_angs.resize(chain_.getNrOfJoints());

            for (unsigned int i = 0; i < chain_.getNrOfJoints(); i++)
            {
                target_joint_angs[i] = q_target_(i);
            }
            move_group_->setJointValueTarget(target_joint_angs);
            
            // move the robot using moveJoints() if IK succeeds
            if (moveJoints())
            {
                // compute error metrics if that succeeds
                computeErrorMetrics();
            }
            RCLCPP_INFO(this->get_logger(), "Inverse kinematics successful!");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Inverse kinematics failed!");
        }

    }
}

bool RobotKinValidation::moveJoints()
{
    // if move plan succeeded, move robot
    if (move_group_->plan(plan_) == moveit::core::MoveItErrorCode::SUCCESS)
    {
        move_group_->execute(plan_);
        return true;
    }
    else // print error otherwise
    {
        RCLCPP_INFO(this->get_logger(), "Plan failed");
        return false;
    }

}

void RobotKinValidation::computeErrorMetrics()
{
    
    try
    {
        // not entirely sure of the names that should be used in the lookupTransform
        // can double check / troubleshoot this first when needed
        geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(ee_frame_, tracked_frame_, tf2::TimePointZero);

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
        );

        translational_errors_[circle_idx_] = computeTranslationMetric(translation);
        rotational_errors_[circle_idx_] = computeRotationMetric(rotation);

        RCLCPP_INFO(
            this->get_logger(),
            "Point %zu | Translation error: %.6f m | Rotation error: %.6f deg",
            circle_idx_,
            translational_errors_[circle_idx_],
            rotational_errors_[circle_idx_]
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

void RobotKinValidation::robotDescriptionCallback(const std_msgs::msg::String & msg)
{
    if (!kdl_parser::treeFromString(msg.data, tree_))
    {
        RCLCPP_ERROR(this->get_logger(), "Couldn't construct KDL tree");
        return;
    }
 
    // Get kinematic chain — change link names to match your robot's URDF
    if (!tree_.getChain("LINK0 NAME", "EE LINK ", chain_)) //replace with meca500 link names
    {
        RCLCPP_ERROR(this->get_logger(), "Couldn't get kinematic chain");
        return;
    }
 
    // Size the joint array to match the chain
    joint_angles_.resize(chain_.getNrOfJoints());
 
    // Create IK solver
    ik_solver_ = std::make_unique<KDL::ChainIkSolverPos_LMA>(chain_);
    solver_ready_ = true;
 
    RCLCPP_INFO(this->get_logger(), "KDL solver ready (%u joints).", chain_.getNrOfJoints());
}

double RobotKinValidation::computeAverageOfArray(const std::array<double, 360> & array)
{
    double sum = 0.0;
    for (size_t i = 0; i < array.size(); i++){
        sum += array[i];
    }
    return sum / 360.0;
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
