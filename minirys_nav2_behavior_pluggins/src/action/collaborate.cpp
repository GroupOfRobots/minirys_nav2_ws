#include <string>
#include <memory>

#include "action/collaborate.hpp"
#include <nav2_costmap_2d/costmap_2d.hpp>
// #include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/obstacle_layer.hpp"
#include <cmath>
#include <random>

namespace minirys_nav2_behavior_pluggins
{

CollaborateAction::CollaborateAction(const std::string& action_name,
                                     const BT::NodeConfiguration& conf):
                                     BT::ActionNodeBase(action_name, conf)
{
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    RCLCPP_INFO(node_->get_logger(), "Starting Collaborate BT node !!!!!!!!!!!!!!!!!!");
    nodeNamespace_ = node_->get_namespace();
    nodeNamespace_ = extractFirstNamespace(nodeNamespace_);
}

BT::NodeStatus CollaborateAction::tick()
{
    std::string collaborator_namespace;
    getInput("collaborators", collaborator_namespace);
    if (collaborator_namespace.empty())
    {
        RCLCPP_INFO(node_->get_logger(), "EMPTY collaborators_namespace SO SUCCESS");
        return BT::NodeStatus::SUCCESS;
    }
    if (std::find(this->collaborators_namespaces_.begin(), this->collaborators_namespaces_.end(), collaborator_namespace) == this->collaborators_namespaces_.end())
    {
        std::string status_topic = "/" + collaborator_namespace + "/navigate_to_pose/_action/status";
        this->collaborators_status_sub_[collaborator_namespace] = node_->create_subscription<action_msgs::msg::GoalStatusArray>(
            status_topic, 10,
            [this, collaborator_namespace] (action_msgs::msg::GoalStatusArray msg)
                {this->receiveCollaboratorStatus(msg, collaborator_namespace);});

        std::string move_pose_topic = "/" + collaborator_namespace + "/move_from_pose";
        this->collaborators_move_pose_cli_[collaborator_namespace] = node_->create_client<minirys_msgs::srv::MoveFromPose>(
            move_pose_topic);

        this->collaborators_namespaces_.emplace_back(collaborator_namespace);
    }

    if (this->collaborators_nav_status[collaborator_namespace])
    {
        RCLCPP_INFO(node_->get_logger(), "Robot [%s] is navigating to a goal. No need to request", collaborator_namespace.c_str());
        return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_INFO(node_->get_logger(), "Robot [%s] is stationary.", collaborator_namespace.c_str());
    RCLCPP_INFO(node_->get_logger(), "SENDING goal pose in order to collaborate");
    geometry_msgs::msg::PoseStamped current_goal_pose;
    getInput("goal", current_goal_pose);
    auto request = std::make_shared<minirys_msgs::srv::MoveFromPose::Request>();
    request->pose_stamped = current_goal_pose;
    request->robot_namespace = this->nodeNamespace_;

    // Send the request and handle the response
    auto result = collaborators_move_pose_cli_[collaborator_namespace]->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        if (result.get()->free_space_found)
        {
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            return BT::NodeStatus::FAILURE;
        }
    }

    return BT::NodeStatus::RUNNING;

}

void CollaborateAction::receiveCollaboratorStatus(action_msgs::msg::GoalStatusArray msg, std::string collaborator_namespace)
{
    for (const auto &status : msg.status_list)
    {
        if (status.status == action_msgs::msg::GoalStatus::STATUS_EXECUTING)
        {
            this->collaborators_nav_status[collaborator_namespace] = true;
        }
        else if (status.status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED or
                 status.status == action_msgs::msg::GoalStatus::STATUS_CANCELED or
                 status.status == action_msgs::msg::GoalStatus::STATUS_ABORTED)
        {
            this->collaborators_nav_status[collaborator_namespace] = false;
        }
    }
}

std::string CollaborateAction::extractFirstNamespace(const std::string &full_namespace)
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

} // namespace minirys_nav2_behavior_pluggins


#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<minirys_nav2_behavior_pluggins::CollaborateAction>("Collaborate");
}