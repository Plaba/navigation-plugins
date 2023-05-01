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


#include "right_left_planner/right_left_algorithm.hpp"
#include "right_left_planner/path_segment.hpp"
#include "rclcpp/rclcpp.hpp"
#include <algorithm>
#include <vector>
#include <boost/geometry/index/rtree.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry.hpp>

namespace right_left_planner {

namespace bgi = boost::geometry::index;

RightLeftAlgorithm::RightLeftAlgorithm(int xs, int ys) {
    // goal and start
    goal[0] = goal[1] = 0;
    start[0] = start[1] = 0;
    rtree = RTree();

    costarr = new CellType[xs * ys];

    for (int i = 0; i < xs * ys; i++) {
        costarr[i] = CellType::FREE_UNVISITED;
    }

    nx = xs;
    ny = ys;
}

RightLeftAlgorithm::~RightLeftAlgorithm() {
    if (costarr) {
        delete[] costarr;
    }
}

void RightLeftAlgorithm::setNavArr(int xs, int ys) {
    if (costarr) {
        delete[] costarr;
    }

    costarr = new CellType[xs * ys];

    for (int i = 0; i < xs * ys; i++) {
        costarr[i] = CellType::FREE_UNVISITED;
    }

    nx = xs;
    ny = ys;
}

CellType RightLeftAlgorithm::getCell(int x, int y) const {
    if (costarr) {
        return costarr[x + y * nx];
    }
    return CellType::OBSTACLE_UNVISITED;
}

LineString RightLeftAlgorithm::calculatePlan() {

    prepare();

    while (!result->completed) {
        auto next = result->visit(std::numeric_limits<double>::max(), rtree, false);
        result = std::make_shared<PathStep>(next);
    }

    LineString path = LineString();

    std::shared_ptr<PathSegment> current_segment = result->next;

    Point p = current_segment->segment.first;
    PointDouble pd;

    pd.set<0>(((double) p.x()) / 2);
    pd.set<1>(((double) p.y()) / 2);

    path.push_back(pd);

    while (current_segment != nullptr) {
        p = current_segment->segment.second;
        pd.set<0>(((double) p.x()) / 2);
        pd.set<1>(((double) p.y()) / 2);

        if (current_segment->next == nullptr) {
            break;
        }
        result = current_segment->next;
    }

    return path;

}

void RightLeftAlgorithm::prepare() {
    rtree.clear();

    segments = getCurrentCostmapLines();

    for (auto segment: segments) {
        rtree.insert(segment);
    }

    PathSegment start_segment = PathSegment();
    start_segment.segment = Segment(Point(2 * start[0], 2 * start[1]), Point(2 * goal[0], 2 * goal[1]));
    result = std::make_shared<PathStep>();
    result->next = std::make_shared<PathSegment>(start_segment);
}

std::shared_ptr<PathStep> RightLeftAlgorithm::calculateStep() {
    result = std::make_shared<PathStep>(result->visit(std::numeric_limits<double>::max(), rtree, true));
    return result;
}

void RightLeftAlgorithm::setGoal(int *g) {
    goal[0] = g[0];
    goal[1] = g[1];

    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),
                 "[RightLeftAlgorithm] Setting goal to %d,%d\n", goal[0], goal[1]);
}

void RightLeftAlgorithm::setStart(int *start_arr) {
    start[0] = start_arr[0];
    start[1] = start_arr[1];


    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),
                 "[RightLeftAlgorithm] Setting start_arr to %d,%d\n", start_arr[0], start_arr[1]);
}

void RightLeftAlgorithm::setObstacle(int x, int y) {
    costarr[x + y * nx] = CellType::OBSTACLE_UNVISITED;
}

void RightLeftAlgorithm::setFree(int x, int y) {
    costarr[x + y * nx] = CellType::FREE_UNVISITED;
}

void RightLeftAlgorithm::setCostmap(const COSTTYPE *cmap, bool isROS,
                                    bool allow_unknown) {
    CellType *cm = costarr;
    CellType told;
    if (isROS) { // ROS-type cost array
        for (int i = 0; i < ny; i++) {
            int k = i * nx;
            for (int j = 0; j < nx; j++, k++, cmap++, cm++) {
                *cm = CellType::OBSTACLE_UNVISITED;
                if (i == 0 || i == ny - 1 || j == 0 || j == nx - 1) {
                    continue; // don't do borders
                }
                // This transforms the incoming cost values:
                // COST_OBS                 -> COST_OBS (incoming "lethal obstacle")
                // COST_OBS_ROS             -> COST_OBS (incoming "inscribed inflated
                // obstacle") values in range 0 to 252 -> values from COST_NEUTRAL to
                // COST_OBS_ROS.
                told = *cm;
                int v = *cmap;
                if (v < COST_OBS_ROS) {
                    v = COST_NEUTRAL + COST_FACTOR * v;
                    if (v >= COST_OBS) {
                        v = COST_OBS - 1;
                    }
                    *cm = costToCellType(v);
                } else if (v == COST_UNKNOWN_ROS && allow_unknown) {
                    v = COST_OBS - 1;
                    *cm = CellType::FREE_UNVISITED;
                }
                if (reduceToUnvisited(told) != *cm) {
                    changed_cells.emplace(std::tuple<int, CellType>(k, told));
                } else {
                    *cm = told;
                }
            }
        }
    } else { // not a ROS map, just a PGM
        for (int i = 0; i < ny; i++) {
            int k = i * nx;
            for (int j = 0; j < nx; j++, k++, cmap++, cm++) {
                *cm = CellType::OBSTACLE_UNVISITED;
                if (i == 0 || i == ny - 1 || j == 0 || j == nx - 1) {
                    continue; // don't do borders
                }
                told = *cm;
                *cm = CellType::OBSTACLE_UNVISITED;
                if (i < 7 || i > ny - 8 || j < 7 || j > nx - 8) {
                    continue; // don't do borders
                }
                int v = *cmap;
                if (v < COST_OBS_ROS) {
                    v = COST_NEUTRAL + COST_FACTOR * v;
                    if (v >= COST_OBS) {
                        v = COST_OBS - 1;
                    }
                    *cm = costToCellType(v);
                } else if (v == COST_UNKNOWN_ROS) {
                    v = COST_OBS - 1;
                    *cm = costToCellType(v);
                }
                if (reduceToUnvisited(told) != *cm) {
                    changed_cells.emplace(std::tuple<int, CellType>(k, told));
                } else {
                    *cm = told;
                }
            }
        }
    }
}

RectilinearPolygonSmoother::RectilinearPolygonSmoother(int startx, int starty, Direction start_direction)
        : x(2 * startx + 1), y(2 * starty + 1), started(false), dir(start_direction) {}

// TODO add elbow smoothing
void RectilinearPolygonSmoother::visit(StepDirection step) {
    if (step != StepDirection::STRAIGHT) {
        if (!started) {
            started = true;
            first_x = x;
            first_y = y;

            std::cout << "smoother: first point: " << first_x << ", " << first_y << std::endl;
        } else {
            segments.push_back(Segment(Point(base_x, base_y), Point(x, y)));
            if (x == first_x && y == first_y) {
                isdone = true;
            }
        }

        base_x = x;
        base_y = y;
        std::cout << "smoother: base_x: " << base_x << ", " << base_y << std::endl;

    }

    switch (step) {
        case StepDirection::LEFT:
            dir = turnLeft(dir);
            break;
        case StepDirection::RIGHT:
            dir = turnRight(dir);
            break;
        case StepDirection::STRAIGHT:
            break;
    }
    takeStep();
}

void RectilinearPolygonSmoother::takeStep() {
    switch (dir) {
        case Direction::RIGHT:
            x += 2;
            break;
        case Direction::UP:
            y -= 2;
            break;
        case Direction::LEFT:
            x -= 2;
            break;
        case Direction::DOWN:
            y += 2;
            break;
    }
}

std::vector<Segment> RectilinearPolygonSmoother::getSegments() {
    return segments;
}

bool RectilinearPolygonSmoother::isDone() {
    return isdone;
}

std::vector<std::shared_ptr<StoredSegment>> RightLeftAlgorithm::visitPolygon(uint index, Direction direction) {
    uint x = index % nx;
    uint y = index / nx;

    std::cout << "visitPolygon: " << x << ", " << y << std::endl;

    std::vector<std::shared_ptr<StoredSegment>> polygon_segments;
    RectilinearPolygonSmoother smoother(x, y, direction);

    do {
        costarr[index] = CellType::OBSTACLE_VISITED;

        x = index % nx;
        y = index / nx;
        uint new_index;

        uint check_turn_left_index;
        uint check_keep_straight_index;
        switch (direction) {
            case Direction::RIGHT:
                check_turn_left_index = CELL(x + 1, y + 1);
                check_keep_straight_index = CELL(x + 1, y);
                break;
            case Direction::UP:
                check_turn_left_index = CELL(x + 1, y - 1);
                check_keep_straight_index = CELL(x, y - 1);
                break;
            case Direction::LEFT:
                check_turn_left_index = CELL(x - 1, y - 1);
                check_keep_straight_index = CELL(x - 1, y);
                break;
            case Direction::DOWN:
                check_turn_left_index = CELL(x - 1, y + 1);
                check_keep_straight_index = CELL(x, y + 1);
                break;
            default: // should never happen
                return polygon_segments;
        };

        StepDirection step;

        if (isObstacle(costarr[check_turn_left_index])) {
            direction = turnLeft(direction);
            new_index = check_turn_left_index;
            step = StepDirection::LEFT;
        } else if (isObstacle(costarr[check_keep_straight_index])) {
            new_index = check_keep_straight_index;
            step = StepDirection::STRAIGHT;
        } else {
            direction = turnRight(direction);
            new_index = index;
            step = StepDirection::RIGHT;
        }

        smoother.visit(step);

        index = new_index;

    } while (!smoother.isDone());

    auto smoother_segments = smoother.getSegments();
    SegmentMetadata base_metadata = SegmentMetadata();
    base_metadata.polygon_visited_clockwise = std::make_shared<bool>(false);
    base_metadata.polygon_visited_counterclockwise = std::make_shared<bool>(false);

    polygon_segments.push_back(std::make_shared<StoredSegment>(smoother_segments[0], base_metadata));
    for (long unsigned int i = 1; i < smoother_segments.size(); i++) {
        auto segment = smoother_segments[i];
        SegmentMetadata metadata = base_metadata;
        auto new_segment = std::make_shared<StoredSegment>();
        if (polygon_segments.size() > 0) {
            metadata.clockwise_segment = polygon_segments.back();
            polygon_segments.back()->second.counterclockwise_segment = new_segment;
        }
        new_segment->first = segment;
        new_segment->second = metadata;
        polygon_segments.push_back(new_segment);
    }

    std::cout << "finished: polygon_segments.size(): " << polygon_segments.size() << std::endl;

    std::cout << std::endl;

    polygon_segments.back()->second.counterclockwise_segment = polygon_segments[0];

    polygon_segments[0]->second.clockwise_segment = polygon_segments.back();

    return polygon_segments;
}

std::vector<StoredSegment> RightLeftAlgorithm::getCurrentCostmapLines() {
    auto result = std::vector<StoredSegment>();

    std::vector<int> to_check;

    auto start_index = CELL(start[0], start[1]);

    if (costarr[start_index] != CellType::FREE_UNVISITED) {
        return result; // error: Should be caught previously
    }

    to_check.push_back(start_index);

    while (!to_check.empty()) {
        auto current_index = to_check.back();
        to_check.pop_back();

        auto current_cell = costarr[current_index];

        if (current_cell == CellType::FREE_UNVISITED) {

            auto x = current_index % nx;
            auto y = current_index / nx;

            int neighbor_xs[4] = {x - 1, x + 1, x, x};
            int neighbor_ys[4] = {y, y, y - 1, y + 1};


            Direction directions[4] = {Direction::UP, Direction::DOWN, Direction::RIGHT, Direction::LEFT};

            for (int i = 0; i < 4; i++) {

                if (neighbor_xs[i] < 0 || neighbor_xs[i] >= nx || neighbor_ys[i] < 0 || neighbor_ys[i] >= ny) {
                    continue;
                }
                int index = CELL(neighbor_xs[i], neighbor_ys[i]);

                if (costarr[index] == CellType::FREE_UNVISITED) {
                    to_check.push_back(index);
                } else if (costarr[index] == CellType::OBSTACLE_UNVISITED) {
                    for (auto line: visitPolygon(index, directions[i])) {
                        result.push_back(*line);
                    }
                }
            }
            costarr[current_index] = CellType::FREE_VISITED;
        }
    }

    return result;
}


} // namespace right_left_planner
