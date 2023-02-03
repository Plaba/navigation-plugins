#include <rrt_star_planner/planner.hpp>
#include <rrt_star_planner/tree_point_2d.hpp>

#include <cmath>
#include <string>
#include <memory>
#include <nav2_core/exceptions.hpp>
#include <nav2_util/node_utils.hpp>
namespace rrt_star_planner
{

void RRTStarPlanner::configure (
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    node_ = parent.lock();
    name_ = name;
    tf_ = tf;
    costmap_ = costmap_ros->getCostmap();
    global_frame_ = costmap_ros->getGlobalFrameID();   
}

void RRTStarPlanner::cleanup()
{
RCLCPP_INFO(
    node_->get_logger(), "Cleaning up plugin %s of type RRTStarPlanner",
    name_.c_str());
}

void RRTStarPlanner::activate()
{
    RCLCPP_INFO(
        node_->get_logger(), "Activating plugin %s of type RRTStarPlanner",
        name_.c_str());
}

void RRTStarPlanner::deactivate()
{
    RCLCPP_INFO(
        node_->get_logger(), "Deactivating plugin %s of type RRTStarPlanner",
        name_.c_str());
}

nav_msgs::msg::Path RRTStarPlanner::createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal)
{
    /**
    * V ← { xinit }; E ← ∅;
    * for i = 1, ... , n do
    *    xrand ← SampleFree_i;
    *    xnearest ← Nearest( G =( V, E) , xrand);
    *    xnew ← Steer( xnearest, xrand);
    *    if ObtacleFree( xnearest, xnew) then
    *        Xnear ← Near( G = ( V, E) , xnew, min {γRRT∗ ( log( card (V) ) / card (V) )^1/d, η}) ;
    *        V ← V ∪ { xnew };
    *        xmin ← xnearest;
    *        cmin ← Cost( xnearest) + c( Line( xnearest, xnew) );
    *
    *        foreach xnear ∈ Xnear do // Connect along a minimum-cost path
    *           if CollisionFree(xnear, xnew) && Cost(xnear) + c(Line(xnear, xnew)) < cmin then
    *              xmin ← xnear; cmin ← Cost( xnear) + c( Line(xnear, xnew));
    *
    *        E ← E ∪ { ( xmin, xnew) };
    *
    *        foreach xnear ∈ Xnear do // Rewire the tree
    *           if CollisionFree( xnew, xnear) && Cost( xnew ) + c( Line( xnew, xnear) ) < Cost( xnear) then // if the cost is better through the new node
    *             xparent ← Parent( xnear);
    *           E ← ( E \ { ( xparent, xnear) } ) ∪ { ( xnew, xnear) } ;
    * return G = (V, E);
    */
    nav_msgs::msg::Path global_path;
    global_path.header.frame_id = global_frame_;

    // Check if the goal and start are in the same frame

    if (start.header.frame_id != goal.header.frame_id) {
        RCLCPP_ERROR(
            node_->get_logger(),
            "The start and goal pose should be in the same frame. Instead they are in %s and %s",
            start.header.frame_id.c_str(), goal.header.frame_id.c_str());
        return global_path;
    }

	tree_.clear();

    max_iter_ = costmap_->getSizeInCellsX() * costmap_->getSizeInCellsY();
    max_dist_ = costmap_->getResolution() * 10;

    auto root = TreePoint2D(goal.pose.position.x, goal.pose.position.y);
    root.setRoot();

    tree_.insertPoint(root);

    const double volume = costmap_->getSizeInMetersX() * costmap_->getSizeInMetersY();
    const double obstacle_percentage = 0.05;

    const double gamma = std::sqrt(3 * (volume * obstacle_percentage) / (M_PI));

    int count = 0;

    while (count < max_iter_) {
        TreePoint2D random_point = getRandomFreePoint();
        TreePoint2D nearest_point = getNearestPoint(random_point);
        TreePoint2D new_point = getSteeredPoint(nearest_point, random_point);

        if (isObstacleFree(nearest_point, new_point)) {
            const double min_dist = std::min(gamma * std::sqrt(std::log(count) / count), max_dist_);
            count++;

            auto neighborhood = tree_.pointsInNeighborhood(new_point, min_dist);

            new_point.setParent(&nearest_point);
            double min_cost = nearest_point.getCost() + nearest_point.getDistance(new_point);
            
            for(auto neighbor : neighborhood){
                if(isObstacleFree(new_point, neighbor)){
                    double cost = new_point.getCost() + new_point.getDistance(neighbor);
                    if(cost < min_cost){
                        new_point.setParent(&neighbor);
                        min_cost = cost;
                    }
                } else {
					neighborhood.erase(neighbor);
				}
            }

			for(auto neighbor : neighborhood){
				if(min_cost + new_point.getDistance(neighbor) < neighbor.getCost()){
					neighbor.setParent(&new_point);
				}
			}
        }
    }

	TreePoint2D goal_point(start.pose.position.x, start.pose.position.y);
	TreePoint2D nearest_point = getNearestPoint(goal_point);

	if(goal_point.getDistance(nearest_point) < goal_tolerance_){
		goal_point.setParent(&nearest_point);
	} else {
		throw nav2_core::PlannerException("Could not find a path to the goal");
	}

	TreePoint2D *current_point = &goal_point;
	while(!current_point->isRoot()){
		geometry_msgs::msg::PoseStamped pose;

		auto parent = current_point->getParent();

		pose.pose.position.x = current_point->getX();
		pose.pose.position.y = current_point->getY();

		double angle = atan2(
				parent->getY() - current_point->getY(),
				parent->getX() - current_point->getX()
				);

		pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), angle));

		global_path.poses.push_back(pose);
		current_point = parent;
	}

	global_path.poses.push_back(goal);

    global_path.header.stamp = node_->now();
    return global_path;
}

TreePoint2D RRTStarPlanner::getRandomFreePoint() {
    while (true)
    {
        const double x = dis_(gen_) * costmap_->getSizeInMetersX() - costmap_->getOriginX();
        const double y = dis_(gen_) * costmap_->getSizeInMetersY() - costmap_->getOriginY();

        unsigned int mx, my;
        costmap_->worldToMap(x, y, mx, my);

        if (costmap_->getCost(mx, my) <= MAX_FREE_SPACE_COST)
            return {x, y};
    }
    
}

TreePoint2D RRTStarPlanner::getNearestPoint(const TreePoint2D& point) {
    return tree_.nearestNeighbor(point);
}

TreePoint2D RRTStarPlanner::getSteeredPoint(const TreePoint2D& from, const TreePoint2D& to) const {

    const double distance = from.getDistance(to);
    if (distance <= max_dist_) {
        return to;
    }

    const double dx = to.getX() - from.getX();
    const double dy = to.getY() - from.getY();
    const double angle = atan2(dy, dx);

    const double x = from.getX() + max_dist_ * cos(angle);
    const double y = from.getY() + max_dist_ * sin(angle);

    return {x, y};
}

bool RRTStarPlanner::isObstacleFree(const TreePoint2D& from, const TreePoint2D& to) {
    const double distance = from.getDistance(to);
    const double dx = to.getX() - from.getX();
    const double dy = to.getY() - from.getY();
    const double angle = atan2(dy, dx);

    double x = from.getX();
    double y = from.getY();

    const double resolution = costmap_->getResolution();

    const double x_step = resolution * cos(angle);
    const double y_step = resolution * sin(angle);

    for (double i = 0; i < distance; i += resolution) {
        x += x_step;
        y += y_step;

        unsigned int mx, my;
        costmap_->worldToMap(x, y, mx, my);
        if (costmap_->getCost(mx, my) >= MAX_FREE_SPACE_COST) {
            return false;
        }
    }

    return true;
}

}  // namespace rrt_star_planner


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rrt_star_planner::RRTStarPlanner, nav2_core::GlobalPlanner)