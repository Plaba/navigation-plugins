#ifndef TEMPLATE_PLANNER_TEMPLATE_HPP
#define TEMPLATE_PLANNER_TEMPLATE_HPP

#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <nav2_core/global_planner.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav2_util/robot_utils.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>

#include <Eigen/Core>

namespace template_planner
{

class TemplatePlanner : public nav2_core::GlobalPlanner
{
public:
  TemplatePlanner() = default;
  ~TemplatePlanner() = default;

  // plugin configure

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  // plugin cleanup
  void cleanup() override;

  // plugin activate
  void activate() override;

  // plugin deactivate
  void deactivate() override;

  // This method creates path for given start and goal pose.
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:

  Eigen::Vector2d getLaneMidPoint(
    const Eigen::Vector2d & start,
    const Eigen::Vector2d & start_direction,
    const Eigen::Vector2d & end
  );

  // TF buffer
  std::shared_ptr<tf2_ros::Buffer> tf_;

  // node ptr
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

  // Global Costmap
  nav2_costmap_2d::Costmap2D *costmap_;

  // The global frame of the costmap
  std::string global_frame_, name_;

  double interpolation_resolution_;
};

}  // namespace template_planner

#endif  // TEMPLATE_PLANNER_TEMPLATE_HPP
