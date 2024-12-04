#include "nodes/AutonomyNode.hpp"
#include <nav2_costmap_2d/costmap_2d.hpp>
// #include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/obstacle_layer.hpp"

#include <chrono>
#include <functional>

using namespace std::chrono_literals;
using std::placeholders::_1;


AutonomyNode::AutonomyNode(rclcpp::NodeOptions options):
	Node("autonomy_cs", options)
{
    nodeNamespace_ = this->get_namespace();
    nodeNamespace_ = extractFirstNamespace(nodeNamespace_);
    RCLCPP_INFO(this->get_logger(), "Namespapce is '%s'", nodeNamespace_.c_str());

    this->goal_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/" + nodeNamespace_ + "/goal_pose", 10);

    this->robotsNamespacesSubscribtion = this->create_subscription<minirys_msgs::msg::RobotsNamespaces>(
        "/" + nodeNamespace_ + "/robots_namespaces", rclcpp::SystemDefaultsQoS(),
        std::bind(&AutonomyNode::receiveRobotsNamespaces, this, std::placeholders::_1));

    this->costmap_subscription =  this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/" + nodeNamespace_ + "/global_costmap/costmap", rclcpp::SystemDefaultsQoS(),
        std::bind(&AutonomyNode::costmapCallback, this, std::placeholders::_1));

    this->collaborate_service = this->create_service<minirys_msgs::srv::MoveFromPose>(
        "/" + nodeNamespace_ + "/move_from_pose",
        std::bind(&AutonomyNode::findFreeSpace, this, std::placeholders::_1, std::placeholders::_2));

    this->tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer_);

	// this->updateTimer = this->create_wall_timer(period, std::bind(&AutonomyNode::update, this));
}

AutonomyNode::~AutonomyNode() = default;

void AutonomyNode::receiveRobotsNamespaces(const minirys_msgs::msg::RobotsNamespaces::SharedPtr msg)
{
    std::vector<std::string> new_namespaces(msg->namespaces.begin(), msg->namespaces.end());

    if (new_namespaces != this->robots_namespaces_)
    {
        RCLCPP_INFO(this->get_logger(), "Received namespaces of robots has changed.");

        for (const auto &robot_namespace : new_namespaces)
        {
            if (std::find(this->robots_namespaces_.begin(), this->robots_namespaces_.end(), robot_namespace) == this->robots_namespaces_.end())
            {
                this->robots_namespaces_.emplace_back(robot_namespace);
            }
        }

        this->robots_namespaces_ = new_namespaces;
    }
}

void AutonomyNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    this->recent_costmap_ = *msg;
}

std::string AutonomyNode::extractFirstNamespace(const std::string &full_namespace)
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

geometry_msgs::msg::PoseStamped AutonomyNode::getCurrentPoseInMap()
{
    RCLCPP_INFO(this->get_logger(), "Getting variables");
    std::string target_frame = this->nodeNamespace_ + "/base_footprint";
    geometry_msgs::msg::TransformStamped transform_stamped;
    geometry_msgs::msg::PoseStamped pose_stamped;

    try
    {
        // Lookup the transform from map to the robot's base frame
        RCLCPP_INFO(this->get_logger(), "Getting transform_stamped");
        transform_stamped = this->tf_buffer_->lookupTransform(this->nodeNamespace_ + "/map", target_frame, tf2::TimePointZero);

        RCLCPP_INFO(this->get_logger(), "Getting Convert");
        // Convert TransformStamped to PoseStamped
        pose_stamped.header = transform_stamped.header;
        pose_stamped.pose.position.x = transform_stamped.transform.translation.x;
        pose_stamped.pose.position.y = transform_stamped.transform.translation.y;
        pose_stamped.pose.position.z = transform_stamped.transform.translation.z;

        pose_stamped.pose.orientation = transform_stamped.transform.rotation;
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(this->get_logger(), "Could not transform '%s' to 'map': %s", target_frame.c_str(), ex.what());
    }

    return pose_stamped;
}

// Check if the pose is free of obstacles
bool AutonomyNode::isFreeSpace(const geometry_msgs::msg::PoseStamped& pose)
{
  // Get the index in the global costmap grid based on the pose
  auto global_costmap_ = this->recent_costmap_ ;
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

geometry_msgs::msg::PoseStamped AutonomyNode::calculateAlternativePose(const geometry_msgs::msg::PoseStamped &goal_pose,
                                                                       const geometry_msgs::msg::PoseStamped &current_pose,
                                                                       double min_distance)
{
    // Calculate the vector from goal_pose to current_pose
    double vector_x = current_pose.pose.position.x - goal_pose.pose.position.x;
    double vector_y = current_pose.pose.position.y - goal_pose.pose.position.y;

    // Normalize the vector to get the direction
    double magnitude = std::sqrt(vector_x * vector_x + vector_y * vector_y);
    if (magnitude == 0.0) {
    throw std::runtime_error("The goal_pose and current_pose are identical, unable to compute direction.");
    }
    double direction_x = vector_x / magnitude;
    double direction_y = vector_y / magnitude;

    // Calculate the new pose moved by min_distance away from the goal_pose
    double new_pose_x = goal_pose.pose.position.x + direction_x * min_distance;
    double new_pose_y = goal_pose.pose.position.y + direction_y * min_distance;

    // Add the new_pose as the first pose
    geometry_msgs::msg::PoseStamped new_pose;
    new_pose.header.frame_id = this->nodeNamespace_ + "/map";
    new_pose.header.stamp = this->now();
    new_pose.pose.position.x = new_pose_x;
    new_pose.pose.position.y = new_pose_y;
    new_pose.pose.orientation = current_pose.pose.orientation;

    return new_pose;
}

std::vector<geometry_msgs::msg::PoseStamped> AutonomyNode::calculateCirclePoses(const geometry_msgs::msg::PoseStamped &goal_pose,
                                                                                const geometry_msgs::msg::PoseStamped &new_pose)
{
    // Resulting vector of poses
    std::vector<geometry_msgs::msg::PoseStamped> circle_poses;

    circle_poses.push_back(new_pose);

    // Calculate the radius of the circle
    double radius = std::sqrt(std::pow(new_pose.pose.position.x - goal_pose.pose.position.x, 2) +
                            std::pow(new_pose.pose.position.y - goal_pose.pose.position.y, 2));

    // Calculate the starting angle
    double starting_angle = std::atan2(new_pose.pose.position.y - goal_pose.pose.position.y,
                                        new_pose.pose.position.x - goal_pose.pose.position.x);

    // Generate 19 equally distributed poses on the circle, alternating sides
    int num_points = 19; // Remaining points after the new_pose
    for (int i = 0; i < num_points; ++i)
    {
        // Alternate between positive and negative direction
        double angle_offset = ((i + 1) / 2) * (2.0 * M_PI / 20); // Offset distance based on point index
        if (i % 2 != 0)
        {
            angle_offset = -angle_offset;
        }
        double angle = starting_angle + angle_offset;

        // Calculate the position on the circle
        double pose_x = goal_pose.pose.position.x + radius * std::cos(angle);
        double pose_y = goal_pose.pose.position.y + radius * std::sin(angle);

        // Create a PoseStamped object for the current pose
        geometry_msgs::msg::PoseStamped circle_pose;
        circle_pose.header = new_pose.header;
        circle_pose.pose.position.x = pose_x;
        circle_pose.pose.position.y = pose_y;
        circle_pose.pose.orientation = new_pose.pose.orientation;

        // Add the pose to the result
        circle_poses.push_back(circle_pose);
    }

    return circle_poses;
}


void AutonomyNode::findFreeSpace(const std::shared_ptr<minirys_msgs::srv::MoveFromPose::Request> request,
                                 std::shared_ptr<minirys_msgs::srv::MoveFromPose::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Received Pose from robot '%s'", request->robot_namespace.c_str());
    RCLCPP_INFO(this->get_logger(), "Received Pose: [x: %.2f, y: %.2f, z: %.2f]",
                request->pose_stamped.pose.position.x,
                request->pose_stamped.pose.position.y,
                request->pose_stamped.pose.position.z);

    this->current_pose = getCurrentPoseInMap();

    // Simulate processing time
    rclcpp::sleep_for(2s);


    double min_distance = 0.4;
    RCLCPP_INFO(this->get_logger(), "Getting getCurrentPoseInMap");
    this->current_pose = getCurrentPoseInMap();
    RCLCPP_INFO(this->get_logger(), "Current Pose In MAP: [x: %.2f, y: %.2f, z: %.2f]",
                this->current_pose.pose.position.x,
                this->current_pose.pose.position.y,
                this->current_pose.pose.position.z);
    geometry_msgs::msg::PoseStamped new_pose = calculateAlternativePose(request->pose_stamped, this->current_pose, min_distance);
    if (isFreeSpace(new_pose))
    {
        RCLCPP_INFO(this->get_logger(), "Getting publish First Pose");
        this->goal_pose_pub_->publish(new_pose);
        response->free_space_found = true;
        return;
    }
    else
    {
        std::vector<geometry_msgs::msg::PoseStamped> alternative_circle_poses;
        RCLCPP_INFO(this->get_logger(), "Getting calculateCirclePoses");
        alternative_circle_poses = calculateCirclePoses(request->pose_stamped, new_pose);
        for (const auto &alternative_pose : alternative_circle_poses)
        {
            if (isFreeSpace(alternative_pose))
            {
                RCLCPP_INFO(this->get_logger(), "Getting publish alternative");
                this->goal_pose_pub_->publish(alternative_pose);
                response->free_space_found = true;
                return;
            }
        }

    }

    RCLCPP_WARN(this->get_logger(), "No free space found.");
    response->free_space_found = false;

}
