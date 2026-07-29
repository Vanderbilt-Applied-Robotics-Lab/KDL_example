#include <kdl_example/kdl_example.h>

KDLExample::KDLExample() : Node("kdl_example")
{
  // setup subscriber for robot description
  robot_description_sub_ = this->create_subscription<std_msgs::msg::String>(
    "robot_description", 
    rclcpp::QoS(rclcpp::KeepLast(1)).durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL),
    std::bind(&KDLExample::robotDescriptionCallback, this, std::placeholders::_1));
  
  // Joint state publisher
  joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states",10);

  // Goal position subscriber
  goal_pos_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>("goal_position", 10,
    std::bind(&KDLExample::goalPositionCallback, this, std::placeholders::_1));

  // must match the names in the URDF 
  joint_state_msg_.name = {"joint_1", "joint_2", "joint_3" , "joint_4"};
  joint_state_msg_.position.resize(joint_state_msg_.name.size());
}

void KDLExample::robotDescriptionCallback(const std_msgs::msg::String& msg)
{
  // Construct KDL tree from URDF
  const std::string urdf = msg.data;
  kdl_parser::treeFromString(urdf, tree_);

  // Get kinematic chain of the robot
  tree_.getChain("base_link", "J4", chain_); // must match names in URDF

  q_ = KDL::JntArray(chain_.getNrOfJoints());

  // Create IK solver
  ik_solver_ = std::make_unique<KDL::ChainIkSolverPos_LMA>(chain_);
  
}

void KDLExample::goalPositionCallback(const geometry_msgs::msg::PointStamped & msg)
{
  goal_position_ << msg.point.x, msg.point.y, msg.point.z;
  
  solveIK();
}

void KDLExample::solveIK()
{
  if (ik_solver_ !=nullptr) // check if the IK solver is correctly setup
  {
    // setup initial guess for joint angles
    KDL::JntArray q_init(chain_.getNrOfJoints());
    q_init(0) = 0.0; q_init(1) = 0.0; q_init(2) = 0.0; q_init(3) = 0.0; // all zeros

    // Convert Eigen::Vector3d to KDL::Vector
    KDL::Vector kdl_position(goal_position_[0], goal_position_[1], goal_position_[2]);

    KDL::Rotation kdl_rotation = KDL::Rotation::Quaternion(
        goal_orientation_.x(), goal_orientation_.y(), goal_orientation_.z(), goal_orientation_.w()
      );
    
    // Construct the KDL::Frame
    KDL::Frame desired_ee_frame(kdl_rotation, kdl_position);

    // SOLVE IK
    int error = ik_solver_->CartToJnt(q_init, desired_ee_frame, q_);

    if (error == KDL::ChainIkSolverPos_LMA::E_NOERROR) // if no error -> publish joint angles
    {
      // set header
      joint_state_msg_.header.stamp = this->get_clock()->now();
      
      // copy joint data into message
      std::copy(q_.data.begin(), q_.data.end(), joint_state_msg_.position.begin());

      // publish
      joint_pub_->publish(joint_state_msg_);

      RCLCPP_INFO(this->get_logger(), "Inverse kinematics successful!");
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Inverse kinematics failed!");
    }
  }
}

int main(int argc, char** argv)
{
  // initialize the node
  rclcpp::init(argc, argv);
  
  // create instance of class
  auto node = std::make_shared<KDLExample>();
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}