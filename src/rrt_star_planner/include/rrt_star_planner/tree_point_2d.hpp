#ifndef RRT_STAR_PLANNER_TREE_POINT_2D_HPP
#define RRT_STAR_PLANNER_TREE_POINT_2D_HPP

#include "rrt_star_planner/utils/planning_point_2d.hpp"

#include <math.h>

namespace rrt_star_planner {

    class TreePoint2D : public PlanningPoint2D {
    public:
        TreePoint2D(double x, double y);

        const double getCost();
        inline TreePoint2D* getParent() const { return parent_; };
        inline bool isRoot() const { return parent_ == this; };

        void setParent(TreePoint2D* parent);
        void setRoot();
        inline double getDistance(const TreePoint2D& other) const {
            return std::hypot(x_ - other.x_, y_ - other.y_);
        };

    private:
        TreePoint2D* parent_;
        double distance_from_parent_;
    };

} // rrt_star_planner
#endif //RRT_STAR_PLANNER_TREE_POINT_2D_HPP
