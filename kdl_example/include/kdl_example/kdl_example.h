
#ifndef KDL_EXAMPLE
#define KDL_EXAMPLE

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <Eigen/Dense>

class KDLExample : public rclcpp::Node
{
public:
    /**
     * Main constructor
     */
    KDLExample();
    ~KDLExample() = default;
    
private:
    /**
     * Callback for the /robot_description message
     */
    void robotDescriptionCallback(const std_msgs::msg::String& msg);

    /**
     * Callback for the /goal_position message
     */
    void goalPositionCallback(const geometry_msgs::msg::PointStamped & msg);

    /**
     * Solves the IK of the robot given the goal xyz position given in the constructor
     * Publishes the joint values if xyz position is reachable. Gives error otherwise
     */
    void solveIK();

    // subscriber for the robot description
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;

    // publisher for the joint state
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    sensor_msgs::msg::JointState joint_state_msg_;

    // subscriber for the goal position
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_pos_sub_;

    // KDL
    KDL::Tree tree_;
    KDL::Chain chain_;
    std::unique_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_; // inverse kinematics solver
    KDL::JntArray q_; // joint array 

    // goal position
    Eigen::Vector3d goal_position_;

    // goal orientation
    Eigen::Quaterniond goal_orientation_ = Eigen::Quaterniond::Identity(); // setup an identity quaternion (x = 0, y = 0, z = 0, w = 1) 
    
};

#endif // KDL_EXAMPLE