#ifndef MULTIPLE_ROBOTS_LAYER_HPP_
#define MULTIPLE_ROBOTS_LAYER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
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

  bool isPointInPolygon(double x, double y, const std::vector<geometry_msgs::msg::Point> &polygon);

  void drawFootprint(nav2_costmap_2d::Costmap2D & master_grid,
                     const std::vector<geometry_msgs::msg::Point> &footprint,
                     double x, double y, double theta);


  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;

  bool need_recalculation_;

  std::vector<std::string> robots_namespaces_;

  std::unordered_map<std::string, rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr> robots_poses_subscribtions_;

  std::unordered_map<std::string, geometry_msgs::msg::Pose2D> current_robots_poses_;
};

}  // namespace nav2_minirys_costmap_plugin

#endif  // MULTIPLE_ROBOTS_LAYER_HPP_
