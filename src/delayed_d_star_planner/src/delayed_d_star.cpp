// Copyright (c) 2023, Peter Labadorf
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
// Modified for Euclidean-distance computation
//
// Path calculation uses no interpolation when pot field is at max in
//   nearby cells
//
// Path calc has sanity check that it succeeded
//

#include "delayed_d_star_planner/delayed_d_star.hpp"

#include "rclcpp/rclcpp.hpp"
#include <algorithm>

namespace delayed_d_star_planner {
//
// create nav fn buffers
//

DelayedDStar::DelayedDStar(int xs, int ys) {
  // create cell arrays
  costarr = NULL;
  g = NULL;
  gradx = grady = NULL;
  setNavArr(xs, ys);

  // goal and start
  goal[0] = goal[1] = 0;
  start[0] = start[1] = 0;

  // display function
  // displayFn = NULL;
  // displayInt = 0;

  // path buffers
  npathbuf = npath = 0;
  pathx = pathy = NULL;
  pathStep = 0.5;
}

DelayedDStar::~DelayedDStar() {
  if (costarr) {
    delete[] costarr;
  }
  if (g) {
    delete[] g;
  }
  if (rhs) {
    delete[] rhs;
  }
  if (gradx) {
    delete[] gradx;
  }
  if (grady) {
    delete[] grady;
  }
  if (pathx) {
    delete[] pathx;
  }
  if (pathy) {
    delete[] pathy;
  }
}

//
// set goal, start positions for the nav fn
//

void DelayedDStar::setGoal(int *g) {
  goal[0] = g[0];
  goal[1] = g[1];

  restart = true;

  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),
               "[DelayedDStar] Setting goal to %d,%d\n", goal[0], goal[1]);
}

void DelayedDStar::setStart(int *g) {
  oldstart[0] = start[0];
  oldstart[1] = start[1];

  start[0] = g[0];
  start[1] = g[1];

  
  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),
               "[DelayedDStar] Setting start to %d,%d\n", start[0], start[1]);
}

//
// Set/Reset map size
//

void DelayedDStar::setNavArr(int xs, int ys) {
  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),
               "[DelayedDStar] Array is %d x %d\n", xs, ys);

  nx = xs;
  ny = ys;
  ns = nx * ny;

  if (costarr) {
    delete[] costarr;
  }
  if (g) {
    delete[] g;
  }
  if (gradx) {
    delete[] gradx;
  }
  if (grady) {
    delete[] grady;
  }

  costarr = new COSTTYPE[ns]; // cost array, 2d config space
  memset(costarr, 0, ns * sizeof(COSTTYPE));
  g = new float[ns]; // navigation potential array
  gradx = new float[ns];
  grady = new float[ns];
}

//
// set up cost array, usually from ROS
//

void DelayedDStar::setCostmap(const COSTTYPE *cmap, bool isROS,
                              bool allow_unknown) {
  COSTTYPE *cm = costarr;
  COSTTYPE cold;
  if (isROS) { // ROS-type cost array
    for (int i = 0; i < ny; i++) {
      int k = i * nx;
      for (int j = 0; j < nx; j++, k++, cmap++, cm++) {
        // This transforms the incoming cost values:
        // COST_OBS                 -> COST_OBS (incoming "lethal obstacle")
        // COST_OBS_ROS             -> COST_OBS (incoming "inscribed inflated
        // obstacle") values in range 0 to 252 -> values from COST_NEUTRAL to
        // COST_OBS_ROS.
        cold = *cm;
        *cm = COST_OBS;
        int v = *cmap;
        if (v < COST_OBS_ROS) {
          v = COST_NEUTRAL + COST_FACTOR * v;
          if (v >= COST_OBS) {
            v = COST_OBS - 1;
          }
          *cm = v;
        } else if (v == COST_UNKNOWN_ROS && allow_unknown) {
          v = COST_OBS - 1;
          *cm = v;
        }
        if (!restart && cold != *cm) {
          changed_cells.emplace(std::tuple<int, char>(k, cold));
        }
      }
    }
  } else { // not a ROS map, just a PGM
    for (int i = 0; i < ny; i++) {
      int k = i * nx;
      for (int j = 0; j < nx; j++, k++, cmap++, cm++) {
        cold = *cm;
        *cm = COST_OBS;
        if (i < 7 || i > ny - 8 || j < 7 || j > nx - 8) {
          continue; // don't do borders
        }
        int v = *cmap;
        if (v < COST_OBS_ROS) {
          v = COST_NEUTRAL + COST_FACTOR * v;
          if (v >= COST_OBS) {
            v = COST_OBS - 1;
          }
          *cm = v;
        } else if (v == COST_UNKNOWN_ROS) {
          v = COST_OBS - 1;
          *cm = v;
        }
        if (!restart && cold != *cm) {
          changed_cells.emplace(std::tuple<int, char>(k, cold));
        }
      }
    }
  }
}

bool DelayedDStar::calcDelayedDStar() { 
  setupDelayedDStar(true); 
  
  if (restart) {
    setupDelayedDStar();
    computeShortestPathDelayed();
    restart = false;
  } else {
    key_bias += heuristic(start[0] + nx * start[1], oldstart[0] + nx * oldstart[1]);

    for (auto &cell : changed_cells) {
      propagateCostChange(std::get<0>(cell), std::get<1>(cell));
    }
    changed_cells.clear();
    computeShortestPathDelayed();
    bool raise = false;
    do {
      raise = findRaiseStatesOnPath();
    } while (raise);
  }
  return true; 
}

//
// returning values
//

float *DelayedDStar::getPathX() { return pathx; }
float *DelayedDStar::getPathY() { return pathy; }
int DelayedDStar::getPathLen() { return npath; }

// Set up navigation potential arrays for new propagation

void DelayedDStar::setupDelayedDStar(bool keepit) {
  // reset values in propagation arrays
  for (int i = 0; i < ns; i++) {
    g[i] = POT_HIGH;
    rhs[i] = POT_HIGH;
    if (!keepit) {
      costarr[i] = COST_NEUTRAL;
    }
    gradx[i] = grady[i] = 0.0;
  }

  // outer bounds of cost array
  COSTTYPE *pc;
  pc = costarr;
  for (int i = 0; i < nx; i++) {
    *pc++ = COST_OBS;
  }
  pc = costarr + (ny - 1) * nx;
  for (int i = 0; i < nx; i++) {
    *pc++ = COST_OBS;
  }
  pc = costarr;
  for (int i = 0; i < ny; i++, pc += nx) {
    *pc = COST_OBS;
  }
  pc = costarr + nx - 1;
  for (int i = 0; i < ny; i++, pc += nx) {
    *pc = COST_OBS;
  }

  // set goal
  int k = goal[0] + goal[1] * nx;
  initCost(k, 0);

  // find # of obstacle cells
  pc = costarr;
  int ntot = 0;
  for (int i = 0; i < ns; i++, pc++) {
    if (*pc >= COST_OBS) {
      ntot++; // number of cells that are obstacles
    }
  }
}

// initialize a goal-type cost for starting propagation

void DelayedDStar::initCost(int k, float v) { g[k] = v; }

bool DelayedDStar::findRaiseStatesOnPath() {
  int s = start[0] + nx * start[1];
  int sgoal = goal[0] + nx * goal[1];
  bool raise = false;
  bool loop = false;

  while (s != sgoal && loop == false) {
    int x = minSuccessor(s);
    rhs[s] = successorCost(x) + g[x];
    if (g[s] != rhs[s]) {
      updateVertex(s);
      raise = true;
    }
    if (x == s) {
      loop = true;
    } else {
      s = x;
    }
  }

  return raise;
}

void DelayedDStar::computeShortestPathDelayed() {
  int sstart = start[0] + nx * start[1];
  while (true) 
  {
    PQueueCell top = *pq.begin();
    PQueueCell snew = calcKey(sstart);

    int predecessors[8] = {
      top.index - nx - 1, 
      top.index - nx, 
      top.index - nx + 1, 
      top.index - 1, 
      top.index + 1, 
      top.index + nx - 1, 
      top.index + nx, 
      top.index + nx + 1
    };

    if(calcKey(sstart) < top && rhs[sstart] == g[sstart])
    {
      break;
    }
    if(top < snew)
    {
      pq.insert(snew);
    }
    else if(g[top.index] > rhs[top.index])
    {
      g[top.index] = rhs[top.index];
      pq.erase(top);
      for(int x : predecessors)
      {
        rhs[x] = std::min(rhs[x], successorCost(top.index) + g[top.index]);
        updateVertexLower(x);
      }
    } else {
      float gold = g[top.index];
      g[top.index] = POT_HIGH;
      for(int x : predecessors)
      {
        if(rhs[x] == successorCost(top.index) + gold || x == top.index)
        {
          if(x != sstart)
          {
            rhs[x] = successorCost(minSuccessor(x)) + g[minSuccessor(x)];
          }
        }
        updateVertex(x);
      }
    }

  }
}

void DelayedDStar::propagateCostChange(int n, COSTTYPE cold) {
  if (cold > costarr[n]) {
    rhs[n] = std::min(rhs[n], successorCost(n) + g[n]);
  } else if (rhs[n] == cold + g[n]) {
    if (n != goal[0] + goal[1] * nx) {
      rhs[n] = successorCost(minSuccessor(n));
    }
  }
}

void DelayedDStar::updateVertexLower(int n) {
  if (g[n] > rhs[n]) {
    pq.insert(calcKey(n));
  } else if (g[n] == rhs[n]) {
    for (PQueueCell c : pq) {
      if (c.index == n) {
        pq.erase(c);
        break;
      }
    }
  }
}

void DelayedDStar::updateVertex(int n) {
  // if (g[s] != rhs(s))
  //     Insert(U, s, CalculateKey(s));
  // else if (g[s] = rhs(s)) and (s ∈ U)
  //     Remove(U, s);
  if (g[n] != rhs[n]) {
    pq.insert(calcKey(n));
  } else if (g[n] == rhs[n]) {
    for (PQueueCell c : pq) {
      if (c.index == n) {
        pq.erase(c);
        break;
      }
    }
  }
}

//
// Path construction
// Find gradient at array points, interpolate path
// Use step size of pathStep, usually 0.5 pixel
//
// Some sanity checks:
//  1. Stuck at same index position
//  2. Doesn't get near goal
//  3. Surrounded by high potentials
//

int DelayedDStar::calcPath(int n, int *st) 
{
  // test write
  // savemap("test");

  // check path arrays
  if (npathbuf < n) {
    if (pathx) {
      delete[] pathx;
    }
    if (pathy) {
      delete[] pathy;
    }
    pathx = new float[n];
    pathy = new float[n];
    npathbuf = n;
  }

  // set up start position at cell
  // st is always upper left corner for 4-point bilinear interpolation
  if (st == NULL) {
    st = start;
  }
  int stc = st[1] * nx + st[0];

  // set up offset
  float dx = 0;
  float dy = 0;
  npath = 0;

  // go for <n> cycles at most
  for (int i = 0; i < n; i++) {
    // check if near goal
    int nearest_point = std::max(
        0, std::min(nx * ny - 1, stc + static_cast<int>(round(dx)) +
                                     static_cast<int>(nx * round(dy))));
    if (g[nearest_point] < COST_NEUTRAL) {
      pathx[npath] = static_cast<float>(goal[0]);
      pathy[npath] = static_cast<float>(goal[1]);
      return ++npath; // done!
    }

    if (stc < nx || stc > ns - nx) { // would be out of bounds
      RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[PathCalc] Out of bounds");
      return 0;
    }

    // add to path
    pathx[npath] = stc % nx + dx;
    pathy[npath] = stc / nx + dy;
    npath++;

    bool oscillation_detected = false;
    if (npath > 2 && pathx[npath - 1] == pathx[npath - 3] &&
        pathy[npath - 1] == pathy[npath - 3]) {
      RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),
                   "[PathCalc] oscillation detected, attempting fix.");
      oscillation_detected = true;
    }

    int stcnx = stc + nx;
    int stcpx = stc - nx;

    // check for potentials at eight positions near cell
    if (g[stc] >= POT_HIGH || g[stc + 1] >= POT_HIGH ||
        g[stc - 1] >= POT_HIGH || g[stcnx] >= POT_HIGH ||
        g[stcnx + 1] >= POT_HIGH || g[stcnx - 1] >= POT_HIGH ||
        g[stcpx] >= POT_HIGH || g[stcpx + 1] >= POT_HIGH ||
        g[stcpx - 1] >= POT_HIGH || oscillation_detected) {
      RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),
                   "[Path] Pot fn boundary, following grid (%0.1f/%d)", g[stc],
                   npath);

      // check eight neighbors to find the lowest
      int minc = stc;
      int minp = g[stc];
      int st = stcpx - 1;
      if (g[st] < minp) {
        minp = g[st];
        minc = st;
      }
      st++;
      if (g[st] < minp) {
        minp = g[st];
        minc = st;
      }
      st++;
      if (g[st] < minp) {
        minp = g[st];
        minc = st;
      }
      st = stc - 1;
      if (g[st] < minp) {
        minp = g[st];
        minc = st;
      }
      st = stc + 1;
      if (g[st] < minp) {
        minp = g[st];
        minc = st;
      }
      st = stcnx - 1;
      if (g[st] < minp) {
        minp = g[st];
        minc = st;
      }
      st++;
      if (g[st] < minp) {
        minp = g[st];
        minc = st;
      }
      st++;
      if (g[st] < minp) {
        minp = g[st];
        minc = st;
      }
      stc = minc;
      dx = 0;
      dy = 0;

      RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),
                   "[Path] Pot: %0.1f  pos: %0.1f,%0.1f", g[stc],
                   pathx[npath - 1], pathy[npath - 1]);

      if (g[stc] >= POT_HIGH) {
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),
                     "[PathCalc] No path found, high potential");
        // savemap("delayed_d_star_highpot");
        return 0;
      }
    } else { // have a good gradient here
      // get grad at four positions near cell
      gradCell(stc);
      gradCell(stc + 1);
      gradCell(stcnx);
      gradCell(stcnx + 1);

      // get interpolated gradient
      float x1 = (1.0 - dx) * gradx[stc] + dx * gradx[stc + 1];
      float x2 = (1.0 - dx) * gradx[stcnx] + dx * gradx[stcnx + 1];
      float x = (1.0 - dy) * x1 + dy * x2; // interpolated x
      float y1 = (1.0 - dx) * grady[stc] + dx * grady[stc + 1];
      float y2 = (1.0 - dx) * grady[stcnx] + dx * grady[stcnx + 1];
      float y = (1.0 - dy) * y1 + dy * y2; // interpolated y

#if 0
      // show gradients
      RCLCPP_DEBUG(
        rclcpp::get_logger("rclcpp"),
        "[Path] %0.2f,%0.2f  %0.2f,%0.2f  %0.2f,%0.2f  %0.2f,%0.2f; final x=%.3f, y=%.3f\n",
        gradx[stc], grady[stc], gradx[stc + 1], grady[stc + 1],
        gradx[stcnx], grady[stcnx], gradx[stcnx + 1], grady[stcnx + 1],
        x, y);
#endif

      // check for zero gradient, failed
      if (x == 0.0 && y == 0.0) {
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[PathCalc] Zero gradient");
        return 0;
      }

      // move in the right direction
      float ss = pathStep / hypot(x, y);
      dx += x * ss;
      dy += y * ss;

      // check for overflow
      if (dx > 1.0) {
        stc++;
        dx -= 1.0;
      }
      if (dx < -1.0) {
        stc--;
        dx += 1.0;
      }
      if (dy > 1.0) {
        stc += nx;
        dy -= 1.0;
      }
      if (dy < -1.0) {
        stc -= nx;
        dy += 1.0;
      }
    }

    //      ROS_INFO("[Path] Pot: %0.1f  grad: %0.1f,%0.1f  pos: %0.1f,%0.1f\n",
    //      g[stc], x, y, pathx[npath-1], pathy[npath-1]);
  }

  //  return npath;  // out of cycles, return failure
  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),
               "[PathCalc] No path found, path too long");
  // savemap("delayed_d_star_pathlong");
  return 0; // out of cycles, return failure
}


//
// display function setup
// <n> is the number of cycles to wait before displaying,
//     use 0 to turn it off

// void
// DelayedDStar::display(void fn(DelayedDStar * nav), int n)
// {
//   displayFn = fn;
//   displayInt = n;
// }

//
// debug writes
// saves costmap and start/goal
//

// void
// DelayedDStar::savemap(const char * fname)
// {
//   char fn[4096];

//   RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[DelayedDStar] Saving costmap
//   and start/goal points");
//   // write start and goal points
//   snprintf(fn, sizeof(fn), "%s.txt", fname);
//   FILE * fp = fopen(fn, "w");
//   if (!fp) {
//     RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Can't open file %s", fn);
//     return;
//   }
//   fprintf(fp, "Goal: %d %d\nStart: %d %d\n", goal[0], goal[1], start[0],
//   start[1]); fclose(fp);

//   // write cost array
//   if (!costarr) {
//     return;
//   }
//   snprintf(fn, sizeof(fn), "%s.pgm", fname);
//   fp = fopen(fn, "wb");
//   if (!fp) {
//     RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Can't open file %s", fn);
//     return;
//   }
//   fprintf(fp, "P5\n%d\n%d\n%d\n", nx, ny, 0xff);
//   fwrite(costarr, 1, nx * ny, fp);
//   fclose(fp);
// }

} // namespace delayed_d_star_planner
