// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2018 Simbe Robotics
// Copyright (c) 2019 Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef delayed_d_star_planner__NAVFN_PLANNER_HPP_
#define delayed_d_star_planner__NAVFN_PLANNER_HPP_

#include <chrono>
#include <string>
#include <memory>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "right_left_planner/right_left_algorithm.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/geometry_utils.hpp"

namespace right_left_planner
{

class DelayedDStarPlanner : public nav2_core::GlobalPlanner
{
public:
  /**
   * @brief constructor
   */
  DelayedDStarPlanner();

  /**
   * @brief destructor
   */
  ~DelayedDStarPlanner();

  /**
   * @brief Configuring plugin
   * @param parent Lifecycle node pointer
   * @param name Name of plugin map
   * @param tf Shared ptr of TF2 buffer
   * @param costmap_ros Costmap2DROS object
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief Cleanup lifecycle node
   */
  void cleanup() override;

  /**
   * @brief Activate lifecycle node
   */
  void activate() override;

  /**
   * @brief Deactivate lifecycle node
   */
  void deactivate() override;


  /**
   * @brief Creating a plan from start and goal poses
   * @param start Start pose
   * @param goal Goal pose
   * @return nav_msgs::Path of the generated path
   */
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

protected:
  /**
   * @brief Compute a plan given start and goal poses, provided in global world frame.
   * @param start Start pose
   * @param goal Goal pose
   * @param plan Path to be computed
   * @return true if can find the path
   */
  bool makePlan(const geometry_msgs::msg::Pose &start, const geometry_msgs::msg::Pose &goal,
                nav_msgs::msg::Path &plan);


	/**
	 * @brief Remove artifacts at the end of the path - originated from planning on a discretized world
	 * @param goal Goal pose
	 * @param plan Computed path
	 */
  void smoothApproachToGoal(
    const geometry_msgs::msg::Pose & goal,
    nav_msgs::msg::Path & plan);

  /**
   * @brief Compute the squared distance between two points
   * @param p1 Point 1
   * @param p2 Point 2
   * @return double squared distance between two points
   */
  inline double squared_distance(
    const geometry_msgs::msg::Pose & p1,
    const geometry_msgs::msg::Pose & p2)
  {
    double dx = p1.position.x - p2.position.x;
    double dy = p1.position.y - p2.position.y;
    return dx * dx + dy * dy;
  }

  /**
   * @brief Transform a point from world to map frame
   * @param wx double of world X coordinate
   * @param wy double of world Y coordinate
   * @param mx int of map X coordinate
   * @param my int of map Y coordinate
   * @return true if can transform
   */
  bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my);

  /**
   * @brief Transform a point from map to world frame
   * @param mx double of map X coordinate
   * @param my double of map Y coordinate
   * @param wx double of world X coordinate
   * @param wy double of world Y coordinate
   */
  void mapToWorld(double mx, double my, double & wx, double & wy);

  /**
   * @brief Set the corresponding cell cost to be free space
   * @param mx int of map X coordinate
   * @param my int of map Y coordinate
   */
  void clearRobotCell(unsigned int mx, unsigned int my);

  /**
   * @brief Determine if a new planner object should be made
   * @return true if planner object is out of date
   */
  bool isPlannerOutOfDate();

  // Last goal pose
  geometry_msgs::msg::Pose last_goal_;

  // Planner based on ROS1 RightLeftAlgorithm algorithm
  std::unique_ptr<RightLeftAlgorithm> planner_;

  // TF buffer
  std::shared_ptr<tf2_ros::Buffer> tf_;

  // Clock
  rclcpp::Clock::SharedPtr clock_;

  // Logger
  rclcpp::Logger logger_{rclcpp::get_logger("DelayedDStarPlanner")};

  // Global Costmap
  nav2_costmap_2d::Costmap2D * costmap_;

  // The global frame of the costmap
  std::string global_frame_, name_;

  // Whether or not the planner should be allowed to plan through unknown space
  bool allow_unknown_, use_final_approach_orientation_;

  // If the goal is obstructed, the tolerance specifies how many meters the planner
  // can relax the constraint in x and y before failing
  double tolerance_;


  // Subscription for parameter change
  rclcpp::AsyncParametersClient::SharedPtr parameters_client_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;

  /**
   * @brief Callback executed when a paramter change is detected
   * @param event ParameterEvent message
   */
  void on_parameter_event_callback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event);
};

}  // namespace right_left_planner

#endif  // delayed_d_star_planner__NAVFN_PLANNER_HPP_
