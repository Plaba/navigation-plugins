
#include "rrt_star_planner/utils/r_star_tree.hpp"

#include <spatialindex/SpatialIndex.h>

namespace rrt_star_planner {
template<class T>
RStarTree2D<T>::RStarTree2D() {
    properties_ = std::make_unique<Tools::PropertySet>();
    storage_manager_.reset(SpatialIndex::StorageManager::returnMemoryStorageManager(*properties_));
    tree_.reset(SpatialIndex::RTree::returnRTree(*storage_manager_, *properties_));
}

template<class T>
void RStarTree2D<T>::insertPoint(const T point) {
    tree_->insertData(0, nullptr, point.getPoint(), index_id_);
    points_.insert(std::make_pair(index_id_, point));
    index_id_++;
}

template<class T>
T RStarTree2D<T>::nearestNeighbor(const T point) {
    NearestNeighborVisitor visitor;
    tree_->nearestNeighborQuery(1, point.getPoint(), visitor);
    return points_.at(visitor.getNearestNeighborId());
}

template<class T>
std::unordered_set<T, std::hash<PlanningPoint2D>> RStarTree2D<T>::pointsInNeighborhood(T point, double radius) {
    PointsInNeighborhoodVisitor visitor;
    SpatialIndex::Ball ball(radius, point.getPoint());

    tree_->containsWhatQuery(ball, visitor);
	std::unordered_set<T, std::hash<PlanningPoint2D>> points;
    for (auto id: visitor.getPointsIds()) {
        points.insert(points_.at(id));
    }
    return points;
}

template<class T>
void RStarTree2D<T>::clear() {
	storage_manager_.reset(SpatialIndex::StorageManager::returnMemoryStorageManager(*properties_));
	tree_.reset(SpatialIndex::RTree::returnRTree(*storage_manager_, *properties_));
    points_.clear();
    index_id_ = 0;
}

void NearestNeighborVisitor::visitNode(const SpatialIndex::INode &in) {}

void NearestNeighborVisitor::visitData(const SpatialIndex::IData &in) {
    nearest_neighbor_id_ = in.getIdentifier();
}

void NearestNeighborVisitor::visitData(std::vector<const SpatialIndex::IData *> &v) {
    nearest_neighbor_id_ = v[0]->getIdentifier();
}

void PointsInNeighborhoodVisitor::visitNode(const SpatialIndex::INode &in) {}

void PointsInNeighborhoodVisitor::visitData(const SpatialIndex::IData &in) {
    points_ids_.push_back(in.getIdentifier());
}

void PointsInNeighborhoodVisitor::visitData(std::vector<const SpatialIndex::IData *> &v) {
    for (auto data: v) {
        points_ids_.push_back(data->getIdentifier());
    }
}

} // rrt_star_planner