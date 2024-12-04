// #ifndef MINIRYS_NAV2_BEHAVIOR_PLUGGINS__ACTION__COLLABORATE_HPP_
// #define MINIRYS_NAV2_BEHAVIOR_PLUGGINS__ACTION__COLLABORATE_HPP_

#include <string>
#include <vector>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include <action_msgs/msg/goal_status_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
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
                BT::InputPort<std::string>("collaborators", "List of robots to collaborate"),
                BT::InputPort<geometry_msgs::msg::PoseStamped>("collaborators_poses", "Pose of collaborator")};
    }

    std::string extractFirstNamespace(const std::string &full_namespace);

    void receiveCollaboratorStatus(action_msgs::msg::GoalStatusArray msg, std::string collaborator_namespace);

    void receiveCollaboratorCostmap(nav_msgs::msg::OccupancyGrid msg, std::string collaborator_namespace);

    bool isFreeSpace(const geometry_msgs::msg::PoseStamped& pose, std::string collaborator);

    geometry_msgs::msg::PoseStamped generateRandomFreePose(const geometry_msgs::msg::PoseStamped& current_pose, double radius, double min_offset, std::string collaborator);

private:
    rclcpp::Node::SharedPtr node_;
    std::string nodeNamespace_;
    std::vector<std::string> collaborators_namespaces_;
    std::unordered_map<std::string, bool> collaborators_nav_status;
    std::unordered_map<std::string, nav_msgs::msg::OccupancyGrid> collaborators_costmap;
    std::unordered_map<std::string, rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr> collaborators_status_sub_;
    std::unordered_map<std::string, rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr> collaborators_costmap_sub_;
    std::unordered_map<std::string, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> collaborators_goal_pose_pub_;
    std::unordered_map<std::string, rclcpp::Client<minirys_msgs::srv::MoveFromPose>::SharedPtr> collaborators_move_pose_cli_;
};

}  // namespace minirys_nav2_behavior_pluggins

// #endif  // MINIRYS_NAV2_BEHAVIOR_PLUGGINS__ACTION__COLLABORATE_HPP_