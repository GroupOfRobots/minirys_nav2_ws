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
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<std::string> collaborators_list;
    std::vector<std::string> updated_collaborators_list;
    getInput("collaborators_list", collaborators_list);
    updated_collaborators_list = collaborators_list;
    if (not collaborators_list.empty())
    {
        for (const auto &name : collaborators_list)
        {
            RCLCPP_INFO(node_->get_logger(), "ROBOT [%s] IN LIST.", name.c_str());
        }
    }
    bool overall_collaborators_work_status = false;
    bool overall_collaboration_status = true;
    if (collaborators_list.empty())
    {
        RCLCPP_INFO(node_->get_logger(), "EMPTY collaborators_list SO SUCCESS");
        return BT::NodeStatus::SUCCESS;
    }
    for (const auto &collaborator_namespace : collaborators_list)
    {
        if (std::find(this->collaborators_namespaces_.begin(), this->collaborators_namespaces_.end(), collaborator_namespace) == this->collaborators_namespaces_.end())
        {
            std::string status_topic = "/" + collaborator_namespace + "/navigate_to_pose/_action/status";
            this->collaborators_status_sub_[collaborator_namespace] = node_->create_subscription<action_msgs::msg::GoalStatusArray>(
                status_topic, 10,
                [this, collaborator_namespace] (action_msgs::msg::GoalStatusArray msg)
                    {this->receiveCollaboratorStatus(msg, collaborator_namespace);});

            std::string work_status_topic = "/" + collaborator_namespace + "/is_working";
            this->collaborators_work_status_sub_[collaborator_namespace] = node_->create_subscription<std_msgs::msg::Bool>(
                work_status_topic, 10,
                [this, collaborator_namespace] (std_msgs::msg::Bool msg)
                    {this->receiveCollaboratorWorkStatus(msg, collaborator_namespace);});

            std::string move_pose_topic = "/" + collaborator_namespace + "/move_from_pose";
            this->collaborators_move_pose_cli_[collaborator_namespace] = node_->create_client<minirys_msgs::srv::MoveFromPose>(
                move_pose_topic);

            this->collaborators_namespaces_.emplace_back(collaborator_namespace);
            rclcpp::spin_some(node_); // Process ROS messages asynchronously
        }

        if (this->collaborators_nav_status[collaborator_namespace])
        {
            RCLCPP_INFO(node_->get_logger(), "Robot [%s] is navigating to a goal. No need to request", collaborator_namespace.c_str());
            updated_collaborators_list.erase(std::remove(updated_collaborators_list.begin(), updated_collaborators_list.end(), collaborator_namespace), updated_collaborators_list.end());
            continue;
        }

        if (this->collaborators_work_status[collaborator_namespace])
        {
            RCLCPP_INFO(node_->get_logger(), "Robot [%s] is working in a goal. Need to wait", collaborator_namespace.c_str());
            overall_collaborators_work_status = true;
            updated_collaborators_list.erase(std::remove(updated_collaborators_list.begin(), updated_collaborators_list.end(), collaborator_namespace), updated_collaborators_list.end());
            continue;
        }
    }
    RCLCPP_INFO(node_->get_logger(), "Switchong lists");
    collaborators_list = updated_collaborators_list;

    if(collaborators_list.empty() and !overall_collaborators_work_status)
    {
        RCLCPP_INFO(node_->get_logger(), "All robots in goal pose are navigating");
        return BT::NodeStatus::SUCCESS;
    }

    if(not collaborators_list.empty())
    {
        geometry_msgs::msg::PoseStamped current_goal_pose;
        getInput("goal", current_goal_pose);
        for (const auto &collaborator_namespace : collaborators_list)
        {
            RCLCPP_INFO(node_->get_logger(), "Robot [%s] is stationary and not working.", collaborator_namespace.c_str());
            RCLCPP_INFO(node_->get_logger(), "SENDING goal pose in order to collaborate");
            auto request = std::make_shared<minirys_msgs::srv::MoveFromPose::Request>();
            request->pose_stamped = current_goal_pose;
            request->robot_namespace = this->nodeNamespace_;
            auto result = collaborators_move_pose_cli_[collaborator_namespace]->async_send_request(request);
        }

        // for (const auto &collaborator_namespace : collaborators_list)
        // {
        //     // Send the request and handle the response
        //     if (rclcpp::spin_until_future_complete(node_->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)
        //     {
        //         if (not result.get()->free_space_found)
        //         {
        //             RCLCPP_INFO(node_->get_logger(), "Robot [%s] cannot find free space.", collaborator_namespace.c_str());
        //             overall_collaboration_status = false;
        //         }
        //         RCLCPP_INFO(node_->get_logger(), "Robot [%s] FOUND! free space HAPPY!! Please Continue.", collaborator_namespace.c_str());
        //         continue;

        //     }
        //     else
        //     {
        //         RCLCPP_INFO(node_->get_logger(), "Failed to call move_to_pose service");
        //         continue;
        //     }
        // }
    }
    if(overall_collaborators_work_status or !overall_collaboration_status)
    {
        RCLCPP_INFO(node_->get_logger(), "Failure! Some robots are working or cannot find free space!");
        return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_INFO(node_->get_logger(), "SUCCESS! !");
    return BT::NodeStatus::SUCCESS;

}

void CollaborateAction::receiveCollaboratorStatus(action_msgs::msg::GoalStatusArray msg, std::string collaborator_namespace)
{
    std::lock_guard<std::mutex> lock(mutex_);
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

void CollaborateAction::receiveCollaboratorWorkStatus(std_msgs::msg::Bool msg, std::string collaborator_namespace)
{
    std::lock_guard<std::mutex> lock(mutex_);
    this->collaborators_work_status[collaborator_namespace] = msg.data;
    RCLCPP_INFO(node_->get_logger(), "Robot [%s] work status is [%d].", collaborator_namespace.c_str(), this->collaborators_work_status[collaborator_namespace]);
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