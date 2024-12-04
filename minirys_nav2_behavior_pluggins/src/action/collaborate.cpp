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
    RCLCPP_INFO(node_->get_logger(), "I WAS TICKED");
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

        // std::string contmap_topic = "/" + collaborator_namespace + "/global_costmap/costmap";
        // this->collaborators_costmap_sub_[collaborator_namespace] = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
        //     contmap_topic, 10,
        //     [this, collaborator_namespace] (nav_msgs::msg::OccupancyGrid msg)
        //         {this->receiveCollaboratorCostmap(msg, collaborator_namespace);});

        // std::string goal_topic = "/" + collaborator_namespace + "/goal_pose";
        // this->collaborators_goal_pose_pub_[collaborator_namespace] = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
        //     goal_topic, 10);
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


    // getInput("collaborators_poses", current_collaborator_pose);
    // geometry_msgs::msg::PoseStamped goal_pose = generateRandomFreePose(current_collaborator_pose, 2.0, 1.0, collaborator_namespace);
    // collaborators_goal_pose_pub_[collaborator_namespace]->publish(goal_pose);

    RCLCPP_INFO(node_->get_logger(), "Robot [%s] is stationary.", collaborator_namespace.c_str());
    RCLCPP_INFO(node_->get_logger(), "SENDING goal pose in order to collaborate");
    geometry_msgs::msg::PoseStamped current_goal_pose;
    getInput("goal", current_goal_pose);
    auto request = std::make_shared<minirys_msgs::srv::MoveFromPose::Request>();
    request->pose_stamped = current_goal_pose;  // Example pose data
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

// void CollaborateAction::receiveCollaboratorCostmap(nav_msgs::msg::OccupancyGrid msg, std::string collaborator_namespace)
// {
//     RCLCPP_INFO(node_->get_logger(), "Costmap received");
//     this->collaborators_costmap[collaborator_namespace] = msg;
// }

geometry_msgs::msg::PoseStamped CollaborateAction::generateRandomFreePose(const geometry_msgs::msg::PoseStamped& current_pose,
                                                                          double radius, double min_offset, std::string collaborator)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.header.stamp = node_->now();

  bool found_free_space = false;
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> random_angle(0.0, 2 * M_PI);
  std::uniform_real_distribution<> random_radius(min_offset, radius);

  while (!found_free_space) {
    // Generate random coordinates within a radius
    double angle = random_angle(gen);
    double distance = random_radius(gen);

    double x = current_pose.pose.position.x + distance * cos(angle);
    double y = current_pose.pose.position.y + distance * sin(angle);

    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.orientation.w = 1.0;  // Face forward

    // Check if the random pose is in a free space
    // if (isFreeSpace(pose, collaborator)) {
      found_free_space = true;
    // }
  }

  return pose;
}

// Check if the pose is free of obstacles
bool CollaborateAction::isFreeSpace(const geometry_msgs::msg::PoseStamped& pose, std::string collaborator)
{
  // Get the index in the global costmap grid based on the pose
  auto global_costmap_ = collaborators_costmap[collaborator];
  int grid_x = (pose.pose.position.x - global_costmap_.info.origin.position.x) / global_costmap_.info.resolution;
  int grid_y = (pose.pose.position.y - global_costmap_.info.origin.position.y) / global_costmap_.info.resolution;

  // Ensure the index is within the bounds of the costmap
  if (grid_x < 0 || grid_y < 0 || grid_x >= static_cast<int>(global_costmap_.info.width) || grid_y >= static_cast<int>(global_costmap_.info.height)) {
    return false;
  }

  int index = grid_y * global_costmap_.info.width + grid_x;

  // Check if the cell is free
  if (index < 0 || index >= static_cast<int>(global_costmap_.data.size())) {
    return false;
  }
  // Use nav2_costmap_2d constants to check for obstacles
  int8_t cell_value = global_costmap_.data[index];

  // Return true if the space is free, otherwise false (considering LETHAL_OBSTACLE or INSCRIBED_INFLATED_OBSTACLE as obstacles)
  return cell_value != nav2_costmap_2d::LETHAL_OBSTACLE && cell_value != nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
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