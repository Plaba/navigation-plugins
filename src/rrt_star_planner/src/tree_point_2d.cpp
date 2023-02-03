#include "rrt_star_planner/tree_point_2d.hpp"

#include <math.h>

namespace rrt_star_planner {

    TreePoint2D::TreePoint2D(double x, double y) : PlanningPoint2D(x, y) {}

    const double TreePoint2D::getCost() {
        return distance_from_parent_ + parent_->getCost();
    }

    void TreePoint2D::setParent(TreePoint2D* parent) {
        parent_ = parent;
        distance_from_parent_ = parent->getCost() + getDistance(*parent);
    }

    void TreePoint2D::setRoot() {
        parent_ = this;
        distance_from_parent_ = 0;
    }
} // rrt_star_planner
