#ifndef RIGHT_LEFT_ALGORITHM_HPP
#define RIGHT_LEFT_ALGORITHM_HPP

#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <vector>
#include <algorithm>

#include "types.hpp"
#include "path_segment.hpp"

#include <boost/polygon/polygon.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/index/indexable.hpp>

namespace right_left_planner {
// cost defs
#define COST_UNKNOWN_ROS 255  // 255 is unknown cost
#define COST_OBS 254  // 254 for forbidden regions
#define COST_OBS_ROS 253  // ROS values of 253 are obstacles

#define COST_OBS_THRESHOLD 253  // Threshold for obstacle cost

// delayed_d_star cost values are set to
// COST_NEUTRAL + COST_FACTOR * costmap_cost_value.
// Incoming costmap cost values are in the range 0 to 252.
// With COST_NEUTRAL of 50, the COST_FACTOR needs to be about 0.8 to
// ensure the input values are spread evenly over the output range, 50
// to 253.  If COST_FACTOR is higher, cost values will have a plateau
// around obstacles and the planner will then treat (for example) the
// whole width of a narrow hallway as equally undesirable and thus
// will not plan paths down the center.

#define COST_NEUTRAL 50  // Set this to "open space" value
#define COST_FACTOR 0.8  // Used for translating costs in RightLeftAlgorithm::setCostmap()

// Define the cost type in the case that it is not set. However, this allows
// clients to modify it without changing the file. Arguably, it is better to require it to
// be defined by a user explicitly
#ifndef COSTTYPE
#define COSTTYPE unsigned char  // Whatever is used...
#endif

enum class CellType : COSTTYPE {
  FREE_UNVISITED = 0,
  FREE_VISITED = 1,
  OBSTACLE_UNVISITED = 3,
  OBSTACLE_VISITED = 2,
};

inline bool isObstacle(CellType cell) {
    return (cell == CellType::OBSTACLE_UNVISITED || cell == CellType::OBSTACLE_VISITED);
}

inline bool isVisited(CellType cell) {
    return (cell == CellType::FREE_VISITED || cell == CellType::OBSTACLE_VISITED);
}

enum class Direction {
  UP = 0,
  LEFT = 1,
  DOWN = 2,
  RIGHT = 3,
};

inline Direction turnLeft(Direction direction) {
    switch (direction) {
        case Direction::UP:
            return Direction::RIGHT;
        case Direction::LEFT:
            return Direction::UP;
        case Direction::DOWN:
            return Direction::LEFT;
        case Direction::RIGHT:
            return Direction::DOWN;
    }
    return Direction::UP; // unreachable
}

inline Direction turnRight(Direction direction) {
    switch (direction) {
        case Direction::UP:
            return Direction::LEFT;
        case Direction::LEFT:
            return Direction::DOWN;
        case Direction::DOWN:
            return Direction::RIGHT;
        case Direction::RIGHT:
            return Direction::UP;
    }
    return Direction::UP; // unreachable
}

enum class StepDirection {
  LEFT,
  RIGHT,
  STRAIGHT,
};

class RectilinearPolygonSmoother {
public:
  RectilinearPolygonSmoother(int startx, int starty, Direction start_dir);

  void visit(StepDirection step);

  std::vector<Segment> getSegments();

  bool isDone();

private:
  // RPSState state;
  // bool new_run = false;

  int x, y;
  bool started = false;
  int base_x, base_y;
  int first_x, first_y;
  Direction dir;

  void takeStep();


  // StepDirection previous_step; 
  // StepDirection previous_turn;

  std::vector<Segment> segments;
  bool isdone = false;
};

/**
 * @class RightLeftAlgorithm
 * @brief Navigation function class. Holds buffers for costmap, delayed_d_star map. Maps are pixel-based.
 *  Origin is upper left, x is right, y is down.
 */
class RightLeftAlgorithm {
public:
  /**
   * @brief  Constructs the planner
   * @param nx The x size of the map
   * @param ny The y size of the map
   */
  RightLeftAlgorithm(int nx, int ny);

  ~RightLeftAlgorithm();

  /**
   * @brief  Sets or resets the size of the map
   * @param nx The x size of the map
   * @param ny The y size of the map
   */
  void setNavArr(int nx, int ny);

  int nx, ny;  /**< size of grid, in pixels */

  void setObstacle(int x, int y);

  void setFree(int x, int y);

  CellType getCell(int x, int y) const;

  void prepare();

  LineString calculatePlan();

  std::shared_ptr<PathStep> calculateStep();


#define CELL(x, y) ((x)+(y)*nx)

  /**
   * @brief  Set up the cost array for the planner, usually from ROS
   * @param cmap The costmap
   * @param isROS Whether or not the costmap is coming in in ROS format
   * @param allow_unknown Whether or not the planner should be allowed to plan through
   *   unknown space
   */
  void setCostmap(const COSTTYPE *cmap, bool isROS = true, bool allow_unknown = true);

  /** cell arrays */
  CellType *costarr;  /**< cost array in 2D configuration space */

  std::shared_ptr<PathStep> result;

  std::vector<StoredSegment> segments;

  inline std::vector<StoredSegment> getSegments() {
      return segments;
  }

  /** list of cells that have changed since the last time the planner was run with the same goal */
  std::set<std::tuple<int, CellType>> changed_cells;

  /** Spatial index of stored polygons*/
  RTree rtree;

  inline CellType costToCellType(COSTTYPE cost) const {
      return (cost < COST_OBS_THRESHOLD) ? CellType::FREE_UNVISITED : CellType::OBSTACLE_UNVISITED;
  }

  inline CellType reduceToUnvisited(CellType cell) const {
      return (cell == CellType::FREE_UNVISITED || cell == CellType::FREE_VISITED) ?
             CellType::FREE_UNVISITED :
             CellType::OBSTACLE_UNVISITED;
  }


  /** goal and start positions */
  /**
   * @brief  Sets the goal position for the planner.
   * Note: the navigation cost field computed gives the cost to get to a given point
   * from the goal, not from the start.
   * @param goal the goal position
   */
  void setGoal(int *goal);

  /**
   * @brief  Sets the start_arr position for the planner.
   * Note: the navigation cost field computed gives the cost to get to a given point
   * from the goal, not from the start_arr.
   * @param start_arr the start_arr position
   */
  void setStart(int *start_arr);

  int goal[2];
  int start[2];

  inline void reset() {
      changed_cells.clear();
      rtree.clear();
      segments.clear();
      result = nullptr;

      for (int i = 0; i < nx * ny; i++) {
          costarr[i] = reduceToUnvisited(costarr[i]);
      }
  }

  std::vector<std::shared_ptr<StoredSegment>>
  visitPolygon(uint index, Direction direction);

  std::vector<StoredSegment> getCurrentCostmapLines();
};

}  // namespace right_left_planner

#endif  // RIGHT_LEFT_ALGORITHM_HPP
