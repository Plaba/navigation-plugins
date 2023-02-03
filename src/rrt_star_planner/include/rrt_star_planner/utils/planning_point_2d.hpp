#ifndef UTILS_RRT_STAR_PLANNER_NAV_POINT_HPP
#define UTILS_RRT_STAR_PLANNER_NAV_POINT_HPP

#include <spatialindex/SpatialIndex.h>

namespace rrt_star_planner {
    class PlanningPoint2D{
    public:
        PlanningPoint2D(double x, double y);
        ~PlanningPoint2D() = default;

		bool operator==(const PlanningPoint2D& other) const;

        inline double getX() const { return x_; };
        inline double getY() const { return y_; };

        SpatialIndex::Point getPoint() const;

    protected:
        const double x_;
        const double y_;
    };
} // rrt_star_planner

template<> struct std::hash<rrt_star_planner::PlanningPoint2D> {
public:
	std::size_t operator()(rrt_star_planner::PlanningPoint2D const& s) const noexcept;
};

#endif //UTILS_RRT_STAR_PLANNER_NAV_POINT_HPP