
#ifndef RIGHT_LEFT_PLANNER_PATH_SEGMENT_HPP
#define RIGHT_LEFT_PLANNER_PATH_SEGMENT_HPP
#include "types.hpp"
#include "right_left_algorithm.hpp"
#include "path_segment.hpp"
#include <memory>
#include <utility>
#include <optional>

namespace right_left_planner {

struct PathChoice;
struct PathSegment;

inline bool withinDelta(double a, double b, double delta) {
    return std::abs(a - b) < delta;
}


/**
 * @brief Get the angle between three points.
 * Is positive for a counterclockwise turn and negative for a clockwise turn.
 * @param p1
 * @param p2
 * @param p3
 * @return angle in radians
 */
double getAngle(const Point &p1, const Point &p2, const Point &p3);

/**
 * @brief Invert an angle.
 * @param angle
 * @return inverted angle
 */
inline double invertAngle(double angle) {
    return angle > 0 ? angle - M_PI : angle + M_PI;
}

struct PathStep{
  double cost;
  bool completed;

  std::shared_ptr<PathChoice> choice = nullptr;
  std::shared_ptr<PathSegment> next = nullptr;

  PathStep visit(double worse_cost, const RTree &rtree, bool single_step);
  double updateCost();
  bool updateComplete();
};

struct PathSegment{
  bool is_wrapping;
  bool obstacle_free;
  bool completed;
  bool from_ccw;
  Segment segment;
  std::shared_ptr<PathStep> next = nullptr;

  PathStep visit(double worse_cost, const RTree &rtree, bool single_step);

  PathStep wrapPolygon(std::shared_ptr<StoredSegment> intersection, bool wrap_ccw,
                       std::shared_ptr<PathStep> next_step) const;

  inline double getLength() const {
      return boost::geometry::distance(segment.first, segment.second);
  }

  inline double getCost() const {
      if(next == nullptr)
          return getLength();
      else
          return getLength() + next->cost;
  }
};

struct PathChoice {
  PathStep clockwise_choice;
  PathStep counter_clockwise_choice;

  inline double getCost() const {
      return std::min(clockwise_choice.cost, counter_clockwise_choice.cost);
  }

  const inline PathStep &getStep(bool from_left) const {
      if (from_left) {
          return clockwise_choice;
      } else {
          return counter_clockwise_choice;
      }
  }

    inline PathStep &minStep() {
        if (clockwise_choice.cost < counter_clockwise_choice.cost) {
            return clockwise_choice;
        } else {
            return counter_clockwise_choice;
        }
    }

    inline PathStep &maxStep() {
        if (clockwise_choice.cost < counter_clockwise_choice.cost) {
            return clockwise_choice;
        } else {
            return counter_clockwise_choice;
        }
    }
};
} // namespace right_left_planner

#endif // RIGHT_LEFT_PLANNER_PATH_SEGMENT_HPP
