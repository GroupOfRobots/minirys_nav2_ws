// #ifndef MINIRYS_NAV2_BEHAVIOR_PLUGGINS__ACTION__COLLABORATE_HPP_
// #define MINIRYS_NAV2_BEHAVIOR_PLUGGINS__ACTION__COLLABORATE_HPP_

#include <string>
#include <vector>
#include <mutex>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include <action_msgs/msg/goal_status_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <minirys_msgs/srv/move_from_pose.hpp>


namespace minirys_nav2_behavior_pluggins
{

class CollaborateAction : public BT::ActionNodeBase
{
public:

    CollaborateAction(const std::string& action_name, const BT::NodeConfiguration& conf);

    CollaborateAction() = delete;

    BT::NodeStatus tick() override;

    void halt() override {}

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Destination to plan to"),
                BT::InputPort<std::vector<std::string>>("collaborators_list", " List of robots to collaborate")};
    }

    std::string extractFirstNamespace(const std::string &full_namespace);

    void receiveCollaboratorStatus(action_msgs::msg::GoalStatusArray msg, std::string collaborator_namespace);

    void receiveCollaboratorWorkStatus(std_msgs::msg::Bool msg, std::string collaborator_namespace);

    geometry_msgs::msg::PoseStamped generateRandomFreePose(const geometry_msgs::msg::PoseStamped& current_pose, double radius, double min_offset, std::string collaborator);

private:
    rclcpp::Node::SharedPtr node_;
    std::string nodeNamespace_;
    std::vector<std::string> collaborators_namespaces_;
    std::unordered_map<std::string, bool> collaborators_nav_status;
    std::unordered_map<std::string, bool> collaborators_work_status;
    std::unordered_map<std::string, rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr> collaborators_status_sub_;
    std::unordered_map<std::string, rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr> collaborators_work_status_sub_;
    std::unordered_map<std::string, rclcpp::Client<minirys_msgs::srv::MoveFromPose>::SharedPtr> collaborators_move_pose_cli_;
    std::mutex mutex_;
};

}  // namespace minirys_nav2_behavior_pluggins

// #endif  // MINIRYS_NAV2_BEHAVIOR_PLUGGINS__ACTION__COLLABORATE_HPP_