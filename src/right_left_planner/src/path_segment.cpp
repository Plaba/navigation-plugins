
#include "right_left_planner/types.hpp"
#include "right_left_planner/path_segment.hpp"

#include <vector>
#include <boost/polygon/polygon.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry.hpp>
#include <utility>


namespace right_left_planner {

namespace bgi=boost::geometry::index;

double getAngle(const Point &first, const Point &middle, const Point &last) {
    double ux = middle.x() - first.x();
    double uy = middle.y() - first.y();
    double vx = middle.x() - last.x();
    double vy = middle.y() - last.y();

    double dot_product = ux * vx + uy * vy;
    double cross_product = ux * vy - uy * vx;
    double ssquared = ux * ux + uy * uy;

    double angle = std::atan2(cross_product/ssquared, dot_product/ssquared);

    return angle;
}

double getCost(const PathStep &step)  {
  return step.cost;
}

bool operator<(const PathStep &lhs, const PathStep &rhs){
  return getCost(lhs) < getCost(rhs);
}

bool operator<(const PathSegment &lhs, const PathSegment &rhs){
  return lhs.getCost() < rhs.getCost();
}

double PathStep::updateCost() {
    if (choice != nullptr) {
        return cost = choice->minStep().cost;
    } else {
        return cost = next->getCost();
    }
}

bool PathStep::updateComplete() {
    if (choice != nullptr) {
        return completed = choice->minStep().completed;
    } else {
        return completed = next->completed;
    }
}
PathStep PathStep::visit(double worse_cost, const RTree &rtree, bool single_step = false)
{
    while (!completed && cost < worse_cost) {
        if (choice != nullptr) {
            double other_cost = choice->minStep().cost;

            if (choice->clockwise_choice.cost < choice->counter_clockwise_choice.cost){
                choice->clockwise_choice = choice->clockwise_choice.visit(other_cost, rtree, false);
            } else {
                choice->counter_clockwise_choice = choice->counter_clockwise_choice.visit(other_cost, rtree, false);
            }
        } else {
            auto result = next->visit(worse_cost, rtree, false);
            if (result.choice != nullptr) {
                choice = result.choice;
                next = nullptr;
            } else {
                next = result.next;
            }
        }
        updateCost();
        updateComplete();

        if (single_step) {
            break;
        }
    }

    if (completed && choice != nullptr) {
        return choice->minStep();
    } else {
        return *this;
    } 
}

PathStep PathSegment::visit(double worse_cost, const RTree &rtree, bool single_step = false)
{
    if (!obstacle_free) {
        auto intersection_iter = rtree.qbegin(
            bgi::intersects(segment) && bgi::nearest(segment.first, 1));

        if(intersection_iter != rtree.qend()) {
            PathStep result;

            auto old_next = next;
            auto intersection = intersection_iter->second.clockwise_segment->second.counterclockwise_segment;

            if(!*intersection->second.polygon_visited_clockwise && *intersection->second.polygon_visited_counterclockwise){
                auto right_result = wrapPolygon(intersection, false, next);
                auto left_result = wrapPolygon(intersection, true, next);

                PathChoice choice;
                choice.clockwise_choice = left_result;
                choice.counter_clockwise_choice = right_result;

                result.choice = std::make_shared<PathChoice>(choice);                
            }

            if ( intersection->second.polygon_visited_clockwise) {
                result = wrapPolygon(intersection, true, next);
            } else {
                result = wrapPolygon(intersection, false, next);
            }

            result.updateCost();
            if (result.cost > worse_cost || single_step) {
                return result;
            }
            return result.visit(worse_cost, rtree, single_step);
        } else {
            obstacle_free = true;
            if (next == nullptr) {
                completed = true;
            }
        }
    }

    if (next != nullptr) {
        auto result = next->visit(worse_cost - getLength(), rtree, single_step);
        if (result.choice != nullptr) {
            auto side = result.choice->getStep(from_ccw);
            if (side.next != nullptr) {
                int multiplier = from_ccw ? 1 : -1;
                double angle = multiplier * getAngle(segment.first, segment.second, side.next->segment.first);
                if (angle > 0) {
                    auto new_value = PathStep();
                    auto new_choice = PathChoice();
                    auto other_branch = PathStep();
                    next = std::make_shared<PathStep>(result.choice->getStep(!from_ccw));
                    other_branch.next = std::make_shared<PathSegment>(*this);

                    if (from_ccw) {
                        new_choice.clockwise_choice = side;
                        new_choice.counter_clockwise_choice = other_branch;
                    } else {
                        new_choice.clockwise_choice = result;
                        new_choice.counter_clockwise_choice = other_branch;
                    }

                    new_value.choice = std::make_shared<PathChoice>(new_choice);
                    new_value.updateCost();

                    return new_value;
                }
            }
        } else {
            next = std::make_shared<PathStep>(result);
        }
    }

    auto result = PathStep();
    result.next = std::make_shared<PathSegment>(*this);

    return result;
}

PathStep
PathSegment::wrapPolygon(std::shared_ptr<StoredSegment> intersection, bool wrap_ccw,
                         std::shared_ptr<PathStep> next_step) const {
    std::vector<std::pair<Point, bool>> points = {
            {segment.second, false},
            {segment.first, false},
    };

    auto &curr_intersection_ref = intersection;

    std::vector<std::shared_ptr<StoredSegment>> stored_segments = {
        intersection
    };

    const int multiplier = wrap_ccw? 1 : -1;

    auto next_point = getSide(curr_intersection_ref, wrap_ccw);

    double external_angle = 0;
    double curr_angle_to_goal;
    double total_exterior_angle;

#define GETBACK(n) ((points.rbegin() + (n))->first)

    do {
        bool is_next_wrapping = true;
        while(points.size() > 2 && getAngle(GETBACK(1), GETBACK(0), next_point) > 0) {
            is_next_wrapping = false;
            external_angle -= invertAngle(getAngle(GETBACK(2), GETBACK(1), GETBACK(0)));
            points.pop_back();
            stored_segments.pop_back();
        }
        if (points.size() == 2){
            is_next_wrapping = false;
        }

        external_angle += invertAngle(getAngle(GETBACK(1), GETBACK(0), next_point));
        double angle_from_current_segment_to_goal = invertAngle(getAngle(GETBACK(0), next_point, segment.second));
        double angle_through_goal_to_start = invertAngle(getAngle(next_point, segment.second, segment.first));
        
        points.push_back({next_point, is_next_wrapping});

        curr_intersection_ref = curr_intersection_ref->second.getSide(wrap_ccw);

        next_point = getSide(curr_intersection_ref, wrap_ccw);

        if (curr_intersection_ref->second.getStep(wrap_ccw) != nullptr) {
            next_step = curr_intersection_ref->second.getStep(wrap_ccw);
            break;
        }

        curr_angle_to_goal = getAngle(GETBACK(0), next_point, segment.second);

        stored_segments.push_back(curr_intersection_ref);

        total_exterior_angle = angle_from_current_segment_to_goal
                 + angle_through_goal_to_start
                 + external_angle;
    } while (
        multiplier * curr_angle_to_goal < 0 ||
        !withinDelta(multiplier * total_exterior_angle, 2 * M_PI, 0.1)
    );
    points.push_back({segment.second, false});

    std::shared_ptr<PathStep> &curr_step = next_step;
    while(points.size() > 2) {
        PathSegment curr_segment;

        auto curr_point = GETBACK(1);
        auto next_point_ = GETBACK(0);

        curr_segment.segment = {curr_point, next_point_};
        curr_segment.is_wrapping = points[points.size() - 2].second;
        curr_segment.obstacle_free = curr_segment.is_wrapping;
        curr_segment.completed = false;
        curr_segment.from_ccw = wrap_ccw;
        curr_segment.next = curr_step;
        curr_step = std::make_shared<PathStep>();
        curr_step->next = std::make_shared<PathSegment>(curr_segment);
        curr_step->updateCost();
        points.pop_back();
        if(!stored_segments.empty()) {
            stored_segments.back()->second.setStep(wrap_ccw, curr_step);
            stored_segments.pop_back();
        }

    }

    return *curr_step;
#undef GETBACK

}

} // namespace right_left_planner