#ifndef RIGHT_LEFT_PLANNER_TYPES_HPP
#define RIGHT_LEFT_PLANNER_TYPES_HPP

#include <boost/polygon/polygon.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/index/indexable.hpp> 

namespace right_left_planner {

struct PathStep;

typedef boost::geometry::model::d2::point_xy<int, boost::geometry::cs::cartesian> Point;
typedef boost::geometry::model::d2::point_xy<double> PointDouble;
typedef boost::geometry::model::segment<Point> Segment;
typedef boost::geometry::model::linestring<PointDouble> LineString;

struct SegmentMetadata{
  std::shared_ptr<std::pair<Segment, SegmentMetadata>> clockwise_segment = nullptr;
  std::shared_ptr<std::pair<Segment, SegmentMetadata>> counterclockwise_segment = nullptr;
  std::shared_ptr<PathStep> clockwise_path = nullptr;
  std::shared_ptr<PathStep> counterclockwise_path = nullptr;
  std::shared_ptr<bool> polygon_visited_clockwise;
  std::shared_ptr<bool> polygon_visited_counterclockwise;

  inline std::shared_ptr<std::pair<Segment, SegmentMetadata>> getSide(bool is_ccw) const
  {
      if(is_ccw)
          return counterclockwise_segment;
      return clockwise_segment;
  }

  inline std::shared_ptr<PathStep> getStep(bool is_ccw) const
  {
      if(is_ccw)
          return counterclockwise_path;
      return clockwise_path;
  }

  inline void setStep(bool is_ccw, const std::shared_ptr<PathStep>& step)
  {
      if(is_ccw)
          counterclockwise_path = step;
      else
          clockwise_path = step;
  }
};

/**
 * The first point in the stored segment is the point going in the clockwise direction
 * The second point is the point going counterclockwise
 */
typedef std::pair<Segment, SegmentMetadata> StoredSegment;

typedef std::shared_ptr<StoredSegment> StoredSegmentPtr;

typedef boost::geometry::index::rtree<StoredSegment, boost::geometry::index::rstar<16, 4>> RTree;

inline Point getSide(const Segment& segment, bool is_ccw) {
    if(is_ccw)
        return segment.second;
    return segment.first;
}

inline Point getSide(const StoredSegmentPtr& segment, bool is_ccw) {
    if(is_ccw)
        return segment->first.second;
    return segment->first.first;
}

inline bool operator==(const Point &lhs, const Point &rhs){
  return lhs.x() == rhs.x() && lhs.y() == rhs.y();
}

inline bool operator==(const Segment &lhs, const Segment &rhs){
  return lhs.first == rhs.first && lhs.second == rhs.second;
}

inline bool operator!=(const Point &lhs, const Point &rhs){
  return lhs.x() != rhs.x() || lhs.y() != rhs.y();
}

inline bool operator!=(const Segment &lhs, const Segment &rhs){
  return lhs.first != rhs.first || lhs.second != rhs.second;
}


} // namespace right_left_planner
#endif // RIGHT_LEFT_PLANNER_TYPES_HPP