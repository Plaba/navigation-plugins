#ifndef UTILS_RRT_STAR_PLANNER_R_STAR_TREE_H
#define UTILS_RRT_STAR_PLANNER_R_STAR_TREE_H

#include <unordered_map>
#include <unordered_set>

#include <spatialindex/SpatialIndex.h>
#include <spatialindex/tools/Tools.h>

#include "rrt_star_planner/utils/planning_point_2d.hpp"

namespace rrt_star_planner {

    template <class T>
    class RStarTree2D {
        static_assert(std::is_base_of<PlanningPoint2D, T>::value);
    public:
        RStarTree2D<T>();

        ~RStarTree2D() = default;

        void insertPoint(T point);

        T nearestNeighbor(T point);
        std::unordered_set<T, std::hash<PlanningPoint2D>> pointsInNeighborhood(T point, double radius);

        void clear();
    private:
        std::unique_ptr<Tools::PropertySet> properties_;
        std::unique_ptr<SpatialIndex::IStorageManager> storage_manager_;
        std::unique_ptr<SpatialIndex::ISpatialIndex> tree_;

        SpatialIndex::id_type index_id_ = 0;
        std::unordered_map<SpatialIndex::id_type, T> points_;
    };

    class NearestNeighborVisitor : public SpatialIndex::IVisitor {
    public:
        void visitNode(const SpatialIndex::INode& in) override;
        void visitData(const SpatialIndex::IData& in) override;
        void visitData(std::vector<const SpatialIndex::IData*>& v) override;

        inline SpatialIndex::id_type getNearestNeighborId() const { return nearest_neighbor_id_; };
    private:
        SpatialIndex::id_type nearest_neighbor_id_;
    };

    class PointsInNeighborhoodVisitor : public SpatialIndex::IVisitor {
    public:
        PointsInNeighborhoodVisitor() = default;
        void visitNode(const SpatialIndex::INode& in) override;
        void visitData(const SpatialIndex::IData& in) override;
        void visitData(std::vector<const SpatialIndex::IData*>& v) override;

        std::vector<SpatialIndex::id_type> getPointsIds() const;
    private:
        std::vector<SpatialIndex::id_type> points_ids_;
    };

} // rrt_star_planner

#endif //UTILS_RRT_STAR_PLANNER_R_STAR_TREE_H
