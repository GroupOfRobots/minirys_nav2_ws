#include <string>
#include <memory>

#include "nav2_util/robot_utils.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/node_utils.hpp"

#include "condition/goal_occupancy.hpp"

namespace minirys_nav2_behavior_pluggins
{

GoalOccupancy::GoalOccupancy(const std::string & condition_name,
                             const BT::NodeConfiguration & conf):
                             BT::ConditionNode(condition_name, conf)
{
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    RCLCPP_INFO(node_->get_logger(), "Starting GoalOccupancy BT node !!!!!!!!!!!!!!!!!!");
    callback_group_ = node_->create_callback_group( rclcpp::CallbackGroupType::MutuallyExclusive, false);
    callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

    rclcpp::SubscriptionOptions sub_option;
    sub_option.callback_group = callback_group_;

    nodeNamespace_ = node_->get_namespace();
    nodeNamespace_ = extractFirstNamespace(nodeNamespace_);

    this->robotsNamespacesSubscribtion = node_->create_subscription<minirys_msgs::msg::RobotsNamespaces>(
        "/" + nodeNamespace_ + "/robots_namespaces", rclcpp::SystemDefaultsQoS(),
        std::bind(&GoalOccupancy::receiveRobotsNamespaces, this, std::placeholders::_1));

    this->tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer_);

}

GoalOccupancy::~GoalOccupancy()
{
    RCLCPP_INFO(node_->get_logger(), "Shutting down GoalOccupancy BT node");
    cleanup();
}

BT::NodeStatus GoalOccupancy::tick()
{
    geometry_msgs::msg::PoseStamped goal_pose;
    getInput("goal", goal_pose);
    // RCLCPP_WARN(node_->get_logger(), "GOAL POSE IS x: %f y: %f z: %f",
    //             goal_pose.pose.position.x,  goal_pose.pose.position.y,  goal_pose.pose.position.z);

    if (status() == BT::NodeStatus::IDLE) {
        rclcpp::spin_some(node_);
        return BT::NodeStatus::RUNNING;
    }

    getRobotsPoses();
    std::vector<std::string> collaborators_list;

    for (const auto& [robot_namespace, robot_pose] : robots_transforms_)
    {
        geometry_msgs::msg::PoseStamped other_robot_pose;
        other_robot_pose.pose.position.x = robot_pose.transform.translation.x;
        other_robot_pose.pose.position.y = robot_pose.transform.translation.y;
        // RCLCPP_WARN(node_->get_logger(), "OTHER ROBOT POSE IS x: %f y: %f z: %f",
        //         other_robot_pose.pose.position.x,  other_robot_pose.pose.position.y,  other_robot_pose.pose.position.z);
        double distance = calculateDistance(goal_pose, other_robot_pose);
        // RCLCPP_WARN(node_->get_logger(), "Distance to other robot is %f", distance);
        if (distance < collision_threshold_)
        {
            RCLCPP_WARN(node_->get_logger(), "Goal pose occupied by '%s' robot.", robot_namespace.c_str());
            collaborators_list.emplace_back(robot_namespace);
        }

    }
    if(not collaborators_list.empty())
    {
        setOutput("collaborators_list", collaborators_list);
        return BT::NodeStatus::FAILURE; // Conflict detected
    }
    RCLCPP_WARN(node_->get_logger(), "Goal NOT occupied by any robot.");
    return BT::NodeStatus::SUCCESS; // No conflict
}

void GoalOccupancy::getRobotsPoses()
{
    if (this->robots_namespaces_.empty())
    {
        return;
    }
    for (const auto& ns : this->robots_namespaces_)
    {
        std::string target_frame = ns + "/base_footprint";
        geometry_msgs::msg::TransformStamped transform_stamped;
        try
        {
            transform_stamped = this->tf_buffer_->lookupTransform(ns + "/map", target_frame, tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(node_->get_logger(), "Could not transform '%s' to 'map': %s", target_frame.c_str(), ex.what());
            continue;
        }
        robots_transforms_[ns] = transform_stamped;
    }
}

double GoalOccupancy::calculateDistance(const geometry_msgs::msg::PoseStamped& pose1, const geometry_msgs::msg::PoseStamped& pose2)
{
    return std::sqrt(std::pow(pose1.pose.position.x - pose2.pose.position.x, 2) +
                        std::pow(pose1.pose.position.y - pose2.pose.position.y, 2));
}

void GoalOccupancy::receiveRobotsNamespaces(const minirys_msgs::msg::RobotsNamespaces::SharedPtr msg)
{
    std::vector<std::string> new_namespaces(msg->namespaces.begin(), msg->namespaces.end());

    if (new_namespaces != this->robots_namespaces_)
    {
        RCLCPP_INFO(node_->get_logger(), "Received namespaces of robots has changed.");

        this->robots_namespaces_ = new_namespaces;
    }
}

std::string GoalOccupancy::extractFirstNamespace(const std::string &full_namespace)
{
    // Remove the leading '/'
    std::string trimmed_namespace = full_namespace.substr(1);

    // Find the position of the first '/'
    size_t pos = trimmed_namespace.find('/');

    // Extract the first part of the namespace
    if (pos != std::string::npos)
    {
        return trimmed_namespace.substr(0, pos);
    }
    else
    {
        // If there is no '/', return the whole string
        return trimmed_namespace;
    }
}
}  // namespace minirys_nav2_behavior_pluggins


#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<minirys_nav2_behavior_pluggins::GoalOccupancy>("GoalOccupancy");
}
