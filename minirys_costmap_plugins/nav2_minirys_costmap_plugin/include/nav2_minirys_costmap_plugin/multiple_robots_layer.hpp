#ifndef MULTIPLE_ROBOTS_LAYER_HPP_
#define MULTIPLE_ROBOTS_LAYER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <tf2/impl/utils.h>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <minirys_msgs/msg/robots_namespaces.hpp>
#include "geometry_msgs/msg/pose2_d.hpp"
#include <vector>
#include <string>
#include <unordered_map>

namespace nav2_minirys_costmap_plugin
{

class MultipleRobotsLayer : public nav2_costmap_2d::Layer
{
public:
    MultipleRobotsLayer();

    virtual void onInitialize();
    virtual void updateBounds(
        double robot_x, double robot_y, double robot_yaw, double * min_x,
        double * min_y,
        double * max_x,
        double * max_y);
    virtual void updateCosts(
        nav2_costmap_2d::Costmap2D & master_grid,
        int min_i, int min_j, int max_i, int max_j);

    virtual void reset()
    {
        return;
    }

    virtual void onFootprintChanged();

    virtual bool isClearable() {return false;}


private:
    rclcpp::Subscription<minirys_msgs::msg::RobotsNamespaces>::SharedPtr robotsNamespacesSubscribtion;

    void receiveRobotsNamespaces(const minirys_msgs::msg::RobotsNamespaces::SharedPtr msgIn);

    void receiveRobotPose(const geometry_msgs::msg::Pose2D::SharedPtr msgIn, const std::string &robot_namespace);

    void getRobotsPoses();

    bool isPointInPolygon(double x, double y, const std::vector<geometry_msgs::msg::Point> &polygon);

    std::string extractFirstNamespace(const std::string &full_namespace);

    void drawFootprints(nav2_costmap_2d::Costmap2D &master_grid,
                        const std::vector<geometry_msgs::msg::Point> &footprint,
                        const std::unordered_map<std::string, geometry_msgs::msg::TransformStamped> &poses,
                        const rclcpp::Time &current_time,
                        int min_i, int min_j, int max_i, int max_j);


    double last_min_x_, last_min_y_, last_max_x_, last_max_y_;

    bool need_recalculation_;

    std::vector<std::string> robots_namespaces_;

    std::unordered_map<std::string, geometry_msgs::msg::TransformStamped> current_robots_transforms_;

    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::mutex pose_mutex_;
};

}  // namespace nav2_minirys_costmap_plugin

#endif  // MULTIPLE_ROBOTS_LAYER_HPP_
