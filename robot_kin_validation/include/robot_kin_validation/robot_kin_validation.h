#ifndef ROBOT_KIN_VALIDATION
#define ROBOT_KIN_VALIDATION

#include <rclcpp/rclcpp.hpp>
// move-it includes
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// lookup transform includes
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

// kdl includes
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <Eigen/Dense>

// Additional includes
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <std_msgs/msg/string.hpp>
#include <cmath>
#include <array>
#include <vector>
#include <memory>
#include <string>

class RobotKinValidation : public rclcpp::Node
{
public:

    RobotKinValidation(rclcpp::NodeOptions options);

    ~RobotKinValidation() = default;

    void initMoveGroup();

    bool mainCircleLoop();

    void solveIKAndMoveRobot(const Eigen::Vector3d & position);

    bool moveJoints();

    void computeErrorMetrics();

    double computeRotationMetric(const Eigen::Quaterniond & rot);

    double computeTranslationMetric(Eigen::Vector3d & pos_vec);
    
private:

    void robotDescriptionCallback(const std_msgs::msg::String& msg);

    double computeAverageOfArray(const std::array<double, 360> & array);

    // subscriber for the robot description
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;

    // Robot path planning
    moveit::planning_interface::MoveGroupInterface::Plan plan_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    
    // members needed for lookup transform
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    // KDL members
    KDL::Tree tree_;
    KDL::Chain chain_;
    std::unique_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_; // inverse kinematics solver
    KDL::JntArray q_init_;
    KDL::JntArray q_target_;
    
    // Circular trajectory members
    Eigen::Vector3d circle_center_;
    double circle_radius_;
    size_t circle_idx_;

    // std::vector<double> for target joint angles
    std::vector<double> target_joint_angs = {0.0, 0.0,  0.0, 0.0, 0.0, 0.0};

    // std::arrays for error metrics  
    std::array<double, 360> translational_errors_;
    std::array<double, 360> rotational_errors_;

    // State variables that could be useful? 
    std::string base_frame_;
    std::string ee_frame_;
    std::string tracked_frame_;
    
};

#endif // ROBOT_KIN_VALIDATION
