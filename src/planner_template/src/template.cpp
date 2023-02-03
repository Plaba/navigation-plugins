#include <template_planner/template.hpp>

#include <cmath>
#include <string>
#include <memory>
#include <nav2_util/node_utils.hpp>

#include <Eigen/Core>

namespace template_planner
{

void TemplatePlanner::configure (
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  // Parameter initialization
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".resolution", rclcpp::ParameterValue(
      0.1));
  node_-> get_parameter(name_ + ".resolution", interpolation_resolution_);
}

void TemplatePlanner::cleanup()
{
  RCLCPP_INFO(
    node_->get_logger(), "Cleaning up plugin %s of type TemplatePlanner",
    name_.c_str());
}

void TemplatePlanner::activate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Activating plugin %s of type TemplatePlanner",
    name_.c_str());
}

void TemplatePlanner::deactivate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Deactivating plugin %s of type TemplatePlanner",
    name_.c_str());
}

nav_msgs::msg::Path TemplatePlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path global_path;
  return global_path;
}

Eigen::Vector2d TemplatePlanner::getLaneMidPoint(
  const Eigen::Vector2d & start,
  const Eigen::Vector2d & start_direction,
  const Eigen::Vector2d & end)
{
  return Eigen::Vector2d::Zero();
}

}  // namespace template_planner


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(template_planner::TemplatePlanner, nav2_core::GlobalPlanner)