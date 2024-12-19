#pragma once

#include <rclcpp/rclcpp.hpp>
#include <minirys_msgs/msg/robots_namespaces.hpp>
#include <minirys_msgs/srv/move_from_pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <vector>
#include <string>
#include <unordered_map>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cmath>
#include <stdexcept>
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <tf2/impl/utils.h>
#include "geometry_msgs/msg/transform_stamped.hpp"

class AutonomyNode: public rclcpp::Node {
public:
	RCLCPP_SMART_PTR_DEFINITIONS(AutonomyNode);

	explicit AutonomyNode(rclcpp::NodeOptions options);

	~AutonomyNode() override;

private:
    bool is_searching{false};
    bool is_working{false};
    std::string nodeNamespace_ = "minirys";
    std::vector<std::string> robots_namespaces_;
    nav_msgs::msg::OccupancyGrid recent_costmap_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    geometry_msgs::msg::PoseStamped current_pose;

	// rclcpp::TimerBase::SharedPtr updateTimer;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr working_status_pub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_subscription;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscription;
    rclcpp::Subscription<minirys_msgs::msg::RobotsNamespaces>::SharedPtr robotsNamespacesSubscribtion;
    rclcpp::Service<minirys_msgs::srv::MoveFromPose>::SharedPtr collaborate_service;

    std::string extractFirstNamespace(const std::string &full_namespace);
    void receiveRobotsNamespaces(const minirys_msgs::msg::RobotsNamespaces::SharedPtr msg);
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    geometry_msgs::msg::PoseStamped getCurrentPoseInMap();
    bool isFreeSpace(const geometry_msgs::msg::PoseStamped& pose);

    geometry_msgs::msg::PoseStamped calculateAlternativePose(const geometry_msgs::msg::PoseStamped &goal_pose,
                                                             const geometry_msgs::msg::PoseStamped &current_pose,
                                                             double min_distance);

    std::vector<geometry_msgs::msg::PoseStamped> calculateCirclePoses(const geometry_msgs::msg::PoseStamped &goal_pose,
                                                                      const geometry_msgs::msg::PoseStamped &new_pose);

    void findFreeSpace(const std::shared_ptr<minirys_msgs::srv::MoveFromPose::Request> request,
                       std::shared_ptr<minirys_msgs::srv::MoveFromPose::Response> response);

};
