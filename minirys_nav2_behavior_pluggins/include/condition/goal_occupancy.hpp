// #ifndef MINIRYS_NAV2_BEHAVIOR_PLUGGINS__CONDITION__GOAL_OCCUPANCY_CONDITION_HPP_
// #define MINIRYS_NAV2_BEHAVIOR_PLUGGINS__CONDITION__GOAL_OCCUPANCY_CONDITION_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <tf2/impl/utils.h>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <minirys_msgs/msg/robots_namespaces.hpp>
#include <vector>
#include <string>
#include <unordered_map>

namespace minirys_nav2_behavior_pluggins
{

class GoalOccupancy : public BT::ConditionNode
{
public:

    GoalOccupancy(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

    GoalOccupancy() = delete;

    ~GoalOccupancy();

    BT::NodeStatus tick() override;

    double calculateDistance(const geometry_msgs::msg::PoseStamped& pose1, const geometry_msgs::msg::PoseStamped& pose2);

    std::string extractFirstNamespace(const std::string &full_namespace);

    void getRobotsPoses();

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Destination to plan to"),};
        // return {BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Destination"),
        //         BT::InputPort<std::string>("global_frame", "Global frame"),
        //         BT::InputPort<std::string>("robot_base_frame", "Robot base frame"),
        //         BT::InputPort<std::unordered_map<std::string, geometry_msgs::msg::PoseStamped>>("other_robot_goals"),
        //         BT::InputPort<minirys_msgs::msg::RobotsNamespaces>("/minirys3/robots_namespaces")};
    }

protected:
    void cleanup(){}

private:
    rclcpp::Node::SharedPtr node_;
    // rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
    rclcpp::Subscription<minirys_msgs::msg::RobotsNamespaces>::SharedPtr robotsNamespacesSubscribtion;

    std::string nodeNamespace_;
    double goal_reached_tol_;
    double transform_tolerance_;
    const double collision_threshold_ = 1.0;
    std::string robot_base_frame_;
    std::vector<std::string> robots_namespaces_;
    std::unordered_map<std::string, geometry_msgs::msg::TransformStamped> robots_transforms_;

    void receiveRobotsNamespaces(const minirys_msgs::msg::RobotsNamespaces::SharedPtr msgIn);


};

}  // namespace minirys_nav2_behavior_pluggins

// #endif  // MINIRYS_NAV2_BEHAVIOR_PLUGGINS__CONDITION__GOAL_OCCUPANCY_CONDITION_HPP_