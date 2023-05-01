// Copyright (c) 2008, Willow Garage, Inc.
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the Willow Garage nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

//
// Navigation function computation
// Uses Dynamic D* method
// Modified for Euclidean-distance computation
//

#ifndef delayed_d_star_planner__NAVFN_HPP_
#define delayed_d_star_planner__NAVFN_HPP_

#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <set>

namespace delayed_d_star_planner
{

// cost defs
#define COST_UNKNOWN_ROS 255  // 255 is unknown cost
#define COST_OBS 254  // 254 for forbidden regions
#define COST_OBS_ROS 253  // ROS values of 253 are obstacles

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
#define COST_FACTOR 0.8  // Used for translating costs in DelayedDStar::setCostmap()

// Define the cost type in the case that it is not set. However, this allows
// clients to modify it without changing the file. Arguably, it is better to require it to
// be defined by a user explicitly
#ifndef COSTTYPE
#define COSTTYPE unsigned char  // Whatever is used...
#endif

// potential defs
#define POT_HIGH 1.0e10  // unassigned cell potential




/**
 * @class DelayedDStar
 * @brief Navigation function class. Holds buffers for costmap, delayed_d_star map. Maps are pixel-based.
 *  Origin is upper left, x is right, y is down.
 */
class DelayedDStar
{
public:
  /**
   * @brief  Constructs the planner
   * @param nx The x size of the map
   * @param ny The y size of the map
   */
  DelayedDStar(int nx, int ny);

  ~DelayedDStar();

  /**
   * @brief  Sets or resets the size of the map
   * @param nx The x size of the map
   * @param ny The y size of the map
   */
  void setNavArr(int nx, int ny);
  int nx, ny, ns;  /**< size of grid, in pixels */

  /**
   * @brief  Set up the cost array for the planner, usually from ROS
   * @param cmap The costmap
   * @param isROS Whether or not the costmap is coming in in ROS format
   * @param allow_unknown Whether or not the planner should be allowed to plan through
   *   unknown space
   */
  void setCostmap(const COSTTYPE * cmap, bool isROS = true, bool allow_unknown = true);

  void updateCostmap(const COSTTYPE * cmap, bool isROS = true, bool allow_unknown = true);

  /**
   * @brief  Calculates a plan using the D* method, returns true if one is found
   * @return True if a plan is found, false otherwise
   */
  bool calcDelayedDStar();

  /**
   * @brief  Accessor for the x-coordinates of a path
   * @return The x-coordinates of a path
   */
  float * getPathX();

  /**
   * @brief  Accessor for the y-coordinates of a path
   * @return The y-coordinates of a path
   */
  float * getPathY();

  /**
   * @brief  Accessor for the length of a path
   * @return The length of a path, 0 if not found
   */
  int getPathLen();

  /**
   * @brief  Gets the cost of the path found the last time a navigation function was computed
   * @return The cost of the last path found
   */
  float getLastPathCost();

  /** cell arrays */
  COSTTYPE * costarr;  /**< cost array in 2D configuration space */
  float * g;  /**< potential array, navigation function potential */
  float * rhs;  /**< potential array, rhs value */

  struct PQueueCell
  {
    int index;  /**< cell index */
    float k_higher;  /**< cell value */
    float k_lower;  /**< cell value */
    PQueueCell(int p, float k_higher, float k_lower) 
      : index(p), k_higher(k_higher), k_lower(k_lower) {}
    bool operator<(const PQueueCell & other) const
    {
      if (k_higher == other.k_higher) {
        return k_lower > other.k_lower;
      }
      return k_higher > other.k_higher;
    }
  };

  /** priority queue */
  std::set<PQueueCell> pq;  /**< priority queue for propagation */
  /** list of cells that have changed since the last time the planner was run with the same goal */
  std::set<std::tuple<int, char>> changed_cells; 

  /** goal and start positions */
  /**
   * @brief  Sets the goal position for the planner.
   * Note: the navigation cost field computed gives the cost to get to a given point
   * from the goal, not from the start.
   * @param goal the goal position
   */
  void setGoal(int * goal);

  /**
   * @brief  Sets the start position for the planner.
   * Note: the navigation cost field computed gives the cost to get to a given point
   * from the goal, not from the start.
   * @param start the start position
   */
  void setStart(int * start);

  int goal[2];
  int start[2];
  int oldstart[2];
  /**
   * @brief  Initialize cell k with cost v for propagation
   * @param k the cell to initialize
   * @param v the cost to give to the cell
   */
  void initCost(int k, float v);

  /** propagation */

  /** 
   * @brief  Run the delayed D* cost update propagation
   * @param n the cell whose cost has changed
   * @param cold the old cost of the cell
  */
  void propagateCostChange(int n, COSTTYPE cold);

  void updateVertexLower(int n);

  void updateVertex(int n);

  /**
   * @brief  Updates the cell at index n using the heuristic
   * @param n The index to update
   */
  void setupDelayedDStar(bool keepit = false);

  bool findRaiseStatesOnPath();

  void computeShortestPathDelayed();

  /** gradient and paths */
  float * gradx, * grady;  /**< gradient arrays, size of potential array */
  float * pathx, * pathy;  /**< path points, as subpixel cell coordinates */
  int npath;  /**< number of path points */
  int npathbuf;  /**< size of pathx, pathy buffers */

  /** D* functions*/
  bool restart = false;
  float key_bias = 0;

  inline float 
  heuristic(int p1, int p2)
  {
    int x1, y1, x2, y2;
    x1 = p1 % nx;
    y1 = p1 / nx;
    x2 = p2 % nx;
    y2 = p2 / nx;

    return COST_NEUTRAL * sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
  }

  inline PQueueCell
  calcKey(int s)
  {
    float higher = std::min(g[s], rhs[s]) + heuristic(start[0]*nx + start[1], s)+key_bias;
    float lower = std::min(g[s], rhs[s]);
    return PQueueCell(s, higher, lower);
  }

  inline int 
  minSuccessor(int stc)
  {
    int stcnx = stc + nx;
    int stcpx = stc - nx;
    int minc = stc;
    int minp = COST_OBS;
    int st = stcpx - 1;
    if (g[st] + successorCost(st) < minp) {minp = g[st] + successorCost(st); minc = st;}
    st++;
    if (g[st] + successorCost(st) < minp) {minp = g[st] + successorCost(st); minc = st;}
    st++;
    if (g[st] + successorCost(st) < minp) {minp = g[st] + successorCost(st); minc = st;}
    st = stc - 1;
    if (g[st] + successorCost(st) < minp) {minp = g[st] + successorCost(st); minc = st;}
    st = stc + 1;
    if (g[st] + successorCost(st) < minp) {minp = g[st] + successorCost(st); minc = st;}
    st = stcnx - 1;
    if (g[st] + successorCost(st) < minp) {minp = g[st] + successorCost(st); minc = st;}
    st++;
    if (g[st] + successorCost(st) < minp) {minp = g[st] + successorCost(st); minc = st;}
    st++;
    if (g[st] + successorCost(st) < minp) {minp = g[st] + successorCost(st); minc = st;}
    
    return minc;
  }

  inline float
  calcRhs(int s)
  {
    if (s == goal[0] + goal[1] * nx) return 0;
    return successorCost(minSuccessor(s));
  }

  inline float 
  successorCost(int s2)
  {
    return COST_NEUTRAL + COST_FACTOR * costarr[s2];
  }

  /**
   * @brief  Calculates the path for at mose <n> cycles
   * @param n The maximum number of cycles to run for
   * @return The length of the path found, 0 if none
   */
  int calcPath(int n, int * st = NULL);

  /**
   * @brief  Calculate gradient at a cell
   * @param n Cell number <n>
   * @return float norm
   */
  float gradCell(int n);  /**< calculates gradient at cell <n>, returns norm */

  float pathStep;  /**< step size for following gradient */

  /** display callback */
  /**< <n> is the number of cycles between updates  */
  // void display(void fn(DelayedDStar * nav), int n = 100);
  // int displayInt;  /**< save second argument of display() above */
  // void (* displayFn)(DelayedDStar * nav);  /**< display function itself */

  /** save costmap */
  /**< write out costmap and start/goal states as fname.pgm and fname.txt */
  // void savemap(const char * fname);
};

}  // namespace delayed_d_star_planner

#endif  // delayed_d_star_planner__NAVFN_HPP_
