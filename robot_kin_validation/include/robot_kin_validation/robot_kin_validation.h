#ifndef ROBOT_KIN_VALIDATION
#define ROBOT_KIN_VALIDATION

#include <rclcpp/rclcpp.hpp>
// move-it includes
// lookup transform includes
// kdl includes
#include <Eigen/Geometry>
#include <std_msgs/msg/string.hpp>
#include <cmath>

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
    // Robot path planning
    // members needed for lookup transform
    // KDL members
    // Circular trajectory members
    // std::vector<double> for target joint angles
    // std::arrays for error metrics  
};

#endif // ROBOT_KIN_VALIDATION