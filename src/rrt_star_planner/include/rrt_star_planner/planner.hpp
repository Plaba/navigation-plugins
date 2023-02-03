#ifndef RRT_STAR_PLANNER_TEMPLATE_HPP
#define RRT_STAR_PLANNER_TEMPLATE_HPP

#include <string>
#include <memory>
#include <random>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <nav2_core/global_planner.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav2_util/robot_utils.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>

#include <Eigen/Core>

#include <spatialindex/SpatialIndex.h>

#include "rrt_star_planner/tree_point_2d.hpp"
#include "rrt_star_planner/utils/r_star_tree.hpp"


namespace rrt_star_planner
{

class RRTStarPlanner : public nav2_core::GlobalPlanner
{
  /**
   * @class rrt_star_planner::RRTStarPlanner
   * @brief Provides a global path planner based on RRT* algorithm, which is decribed in
   * "Sampling-based Algorithms for Optimal Motion Planning" by Sertac Karaman and Emilio Frazzoli
  */
public:
  RRTStarPlanner() = default;
  ~RRTStarPlanner() override = default;

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

    // This method returns a random point in the free space.
    TreePoint2D getRandomFreePoint();

    // This method returns the nearest point in the tree to the given point.
    TreePoint2D getNearestPoint(const TreePoint2D &point);

    // This method returns the closest point to one point within a given distance of the other
    // point.
    TreePoint2D getSteeredPoint(const TreePoint2D &from, const TreePoint2D &to) const;

    // This method returns if the straight line between two points is collision free.
    bool isObstacleFree(const TreePoint2D &point1, const TreePoint2D &point2);

    // TF buffer
    std::shared_ptr<tf2_ros::Buffer> tf_;

    // node ptr
    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

    // Global Costmap
    nav2_costmap_2d::Costmap2D *costmap_;

    // RTree for storing the vertices
    RStarTree2D<TreePoint2D> tree_;

    // The global frame of the costmap
    std::string global_frame_, name_;

    // The maximum distance between two points in the tree
    double max_dist_;

    const uint8_t MAX_FREE_SPACE_COST = 250;

    // The maximum number of iterations
    unsigned int max_iter_;

	double goal_tolerance_ = 0.1;

    // Random number generator
    std::random_device rd_;
    std::mt19937 gen_{rd_()};
    std::uniform_real_distribution<double> dis_{0,1};
};

}  // namespace rrt_star_planner

#endif  // RRT_STAR_PLANNER_TEMPLATE_HPP
