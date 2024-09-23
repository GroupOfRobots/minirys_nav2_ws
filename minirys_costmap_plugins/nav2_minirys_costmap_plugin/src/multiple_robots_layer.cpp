#include "nav2_minirys_costmap_plugin/multiple_robots_layer.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace nav2_minirys_costmap_plugin
{

MultipleRobotsLayer::MultipleRobotsLayer()
: last_min_x_(-std::numeric_limits<float>::max()),
  last_min_y_(-std::numeric_limits<float>::max()),
  last_max_x_(std::numeric_limits<float>::max()),
  last_max_y_(std::numeric_limits<float>::max())
{
}

// This method is called at the end of plugin initialization.
// It contains ROS parameter(s) declaration and initialization
// of need_recalculation_ variable.
void MultipleRobotsLayer::onInitialize()
{
    auto node = node_.lock();
    declareParameter("enabled", rclcpp::ParameterValue(true));
    node->get_parameter(name_ + "." + "enabled", enabled_);

    need_recalculation_ = false;
    current_ = true;

    std::string nodeNamespace = node->get_namespace();
    nodeNamespace = extractFirstNamespace(nodeNamespace);

    this->robotsNamespacesSubscribtion = node->create_subscription<minirys_msgs::msg::RobotsNamespaces>(
            "/" + nodeNamespace + "/robots_namespaces", 10,
            std::bind(&MultipleRobotsLayer::receiveRobotsNamespaces, this, std::placeholders::_1));

    this->tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer_);
    this->timer_ = node->create_wall_timer(std::chrono::milliseconds(100), std::bind(&MultipleRobotsLayer::getRobotsPoses, this));
}

std::string MultipleRobotsLayer::extractFirstNamespace(const std::string &full_namespace)
{
  // Remove the leading '/'
  std::string trimmed_namespace = full_namespace.substr(1);

  // Find the position of the first '/'
  size_t pos = trimmed_namespace.find('/');

  // Extract the first part of the namespace
  if (pos != std::string::npos) {
    return trimmed_namespace.substr(0, pos);
  } else {
    // If there is no '/', return the whole string
    return trimmed_namespace;
  }
}

void MultipleRobotsLayer::receiveRobotsNamespaces(const minirys_msgs::msg::RobotsNamespaces::SharedPtr msg)
{
    std::vector<std::string> new_namespaces(msg->namespaces.begin(), msg->namespaces.end());

    if (new_namespaces != this->robots_namespaces_)
    {
        auto node = node_.lock();
        RCLCPP_INFO(node->get_logger(), "Received namespaces of robots has changed.");

        this->robots_namespaces_ = new_namespaces;
        need_recalculation_ = true;

        // for (const auto &robot_namespace : new_namespaces)
        // {
        //     if (std::find(this->robots_namespaces_.begin(), this->robots_namespaces_.end(), robot_namespace) == this->robots_namespaces_.end())
        //     {
        //         // In order to pass two arguments into callback function we have to use lambda.
        //         // It is because std::bind can handle at most three arguments
        //         // std::string topic_name = "/" + robot_namespace + "/amcl_pose";
        //         // this->robots_poses_subscribtions_[robot_namespace] = node->create_subscription<geometry_msgs::msg::Pose2D>(
        //         //     topic_name, 10,
        //         //     [this, robot_namespace] (geometry_msgs::msg::Pose2D::SharedPtr msg)
        //         //         {this->receiveRobotPose(msg, robot_namespace);});
        //         this->robots_namespaces_.emplace_back(robot_namespace);
        //     }
        // }
    }
}

void MultipleRobotsLayer::getRobotsPoses()
{
    if (this->robots_namespaces_.empty())
    {
        return;
    }
    auto node = node_.lock();
    for (const auto& ns : this->robots_namespaces_)
    {
        std::string target_frame = ns + "/base_footprint";
        geometry_msgs::msg::TransformStamped transform_stamped;
        try
        {
            transform_stamped = this->tf_buffer_->lookupTransform(ns + "/map", target_frame, tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(node->get_logger(), "Could not transform '%s' to 'map': %s", target_frame.c_str(), ex.what());
            continue;
        }
        current_robots_transforms_[ns] = transform_stamped;
    }
}

// The method is called to ask the plugin: which area of costmap it needs to update.
// Inside this method window bounds are re-calculated if need_recalculation_ is true
// and updated independently on its value.
void MultipleRobotsLayer::updateBounds(double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
                                       double * min_x, double * min_y, double * max_x, double * max_y)
{
    if (need_recalculation_)
    {
        last_min_x_ = *min_x;
        last_min_y_ = *min_y;
        last_max_x_ = *max_x;
        last_max_y_ = *max_y;
        // For some reason when I make these -<double>::max() it does not
        // work with Costmap2D::worldToMapEnforceBounds(), so I'm using
        // -<float>::max() instead.
        *min_x = -std::numeric_limits<float>::max();
        *min_y = -std::numeric_limits<float>::max();
        *max_x = std::numeric_limits<float>::max();
        *max_y = std::numeric_limits<float>::max();
        need_recalculation_ = false;
    }
    else
    {
        double tmp_min_x = last_min_x_;
        double tmp_min_y = last_min_y_;
        double tmp_max_x = last_max_x_;
        double tmp_max_y = last_max_y_;
        last_min_x_ = *min_x;
        last_min_y_ = *min_y;
        last_max_x_ = *max_x;
        last_max_y_ = *max_y;
        *min_x = std::min(tmp_min_x, *min_x);
        *min_y = std::min(tmp_min_y, *min_y);
        *max_x = std::max(tmp_max_x, *max_x);
        *max_y = std::max(tmp_max_y, *max_y);
    }
}

// The method is called when footprint was changed.
// Here it just resets need_recalculation_ variable.
void MultipleRobotsLayer::onFootprintChanged()
{
    need_recalculation_ = true;
    RCLCPP_DEBUG(rclcpp::get_logger("nav2_costmap_2d"),
                                    "MultipleRobotsLayer::onFootprintChanged(): num footprint points: %lu",
                                    layered_costmap_->getFootprint().size());
}


bool MultipleRobotsLayer::isPointInPolygon(double x, double y, const std::vector<geometry_msgs::msg::Point> &polygon)
{
    int n = polygon.size();
    bool inside = false;

    for (int i = 0, j = n - 1; i < n; j = i++)
    {
        double xi = polygon[i].x, yi = polygon[i].y;
        double xj = polygon[j].x, yj = polygon[j].y;

        bool intersect = ((yi > y) != (yj > y)) &&
                            (x < (xj - xi) * (y - yi) / (yj - yi) + xi);
        if (intersect)
        {
            inside = !inside;
        }
    }

    return inside;
}

void MultipleRobotsLayer::drawFootprints(nav2_costmap_2d::Costmap2D & master_grid,
                                        const std::vector<geometry_msgs::msg::Point> &footprint,
                                        const std::unordered_map<std::string, geometry_msgs::msg::TransformStamped> &poses,
                                        const rclcpp::Time &current_time,
                                        int min_i, int min_j, int max_i, int max_j)
{

    unsigned char * master_array = master_grid.getCharMap();

    // Precompute the world coordinates for the entire grid
    std::vector<std::vector<tf2::Vector3>> world_points(
    max_j - min_j + 1, std::vector<tf2::Vector3>(max_i - min_i + 1));

    for (int j = min_j; j <= max_j; ++j)
    {
        for (int i = min_i; i <= max_i; ++i)
        {
            double wx, wy;
            master_grid.mapToWorld(i, j, wx, wy);
            world_points[j - min_j][i - min_i] = tf2::Vector3(wx, wy, 0.0);
        }
    }

    for (const auto &pose_pair : poses) {
    const auto &transform_stamped = pose_pair.second;

    // Check if the timestamp is outdated
    rclcpp::Time timestamp(transform_stamped.header.stamp);
    if ((current_time - timestamp).seconds() > 1.0) // Skip if outdated
    {
        continue;
    }

    // Extract the translation and rotation from the TransformStamped
    double x = transform_stamped.transform.translation.x;
    double y = transform_stamped.transform.translation.y;
    tf2::Quaternion q(
        transform_stamped.transform.rotation.x,
        transform_stamped.transform.rotation.y,
        transform_stamped.transform.rotation.z,
        transform_stamped.transform.rotation.w);

    // Create the transformation matrix from the pose
    tf2::Transform transform;
    transform.setOrigin(tf2::Vector3(x, y, 0.0));
    transform.setRotation(q);

    // Transform the footprint points
    std::vector<tf2::Vector3> transformed_footprint;
    for (const auto &point : footprint)
    {
        tf2::Vector3 tf_point(point.x, point.y, 0.0);
        transformed_footprint.push_back(transform * tf_point);
    }

    // Fill the footprint area in the costmap
    for (int j = min_j; j <= max_j; ++j)
    {
        for (int i = min_i; i <= max_i; ++i)
        {
            const tf2::Vector3 &point = world_points[j - min_j][i - min_i];
            tf2::Vector3 transformed_point = transform.inverse() * point;

            if (isPointInPolygon(transformed_point.x(), transformed_point.y(), footprint))
            {
                unsigned int index = master_grid.getIndex(i, j);
                master_array[index] = LETHAL_OBSTACLE; // Draw the footprint
            }
        }
    }
    }
}

void MultipleRobotsLayer::updateCosts(nav2_costmap_2d::Costmap2D & master_grid,
                                      int min_i, int min_j,int max_i, int max_j)
{
    if (!enabled_ || current_robots_transforms_.empty())
    {
        return;
    }

    // Draw the footprint on the costmap
    std::vector<geometry_msgs::msg::Point> footprint = layered_costmap_->getFootprint();
    auto node = node_.lock();
    auto current_time = node->now();
    drawFootprints(master_grid, footprint, current_robots_transforms_, current_time, min_i, min_j, max_i, max_j);
}


}  // namespace nav2_minirys_costmap_plugin

// This is the macro allowing a nav2_minirys_costmap_plugin::MultipleRobotsLayer class
// to be registered in order to be dynamically loadable of base type nav2_costmap_2d::Layer.
// Usually places in the end of cpp-file where the loadable class written.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_minirys_costmap_plugin::MultipleRobotsLayer, nav2_costmap_2d::Layer)
