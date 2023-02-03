#include "rrt_star_planner/utils/planning_point_2d.hpp"

#include <spatialindex/Point.h>

namespace rrt_star_planner {

PlanningPoint2D::PlanningPoint2D(double x, double y) : x_(x), y_(y) {}

bool PlanningPoint2D::operator==(const PlanningPoint2D &other) const {
	return x_ == other.x_ && y_ == other.y_;
}

SpatialIndex::Point PlanningPoint2D::getPoint() const {
    double coords[] = {x_, y_};
    return {coords, 2};
}

} // rrt_star_planner

std::size_t
std::hash<rrt_star_planner::PlanningPoint2D>::operator()(const rrt_star_planner::PlanningPoint2D &s) const noexcept
{
	return std::hash<double>()(s.getX()) ^ std::hash<double>()(s.getY());
}