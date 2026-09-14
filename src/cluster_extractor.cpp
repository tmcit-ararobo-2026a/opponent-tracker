#include "opponent_tracker/cluster_extractor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_set>

namespace opponent_tracker {

ClusterExtractor::ClusterExtractor(const ClusterParams& params) : params_(params) {}

void ClusterExtractor::setParams(const ClusterParams& params)
{
    params_ = params;
}

const ClusterParams& ClusterExtractor::getParams() const
{
    return params_;
}

void ClusterExtractor::setStaticObstacles(const std::vector<StaticObstacle>& obstacles)
{
    static_obstacles_ = obstacles;
}

bool ClusterExtractor::isInsideStaticObstacle(const Eigen::Vector3f& point) const
{
    // Field boundary check
    if (point.x() < params_.field_min_x || point.x() > params_.field_max_x ||
        point.y() < params_.field_min_y || point.y() > params_.field_max_y) {
        return true;  // Outside field
    }

    // Check known static obstacles (with a safety margin)
    constexpr float MARGIN = 0.15f;
    for (const auto& obs : static_obstacles_) {
        const float half_x = obs.size_xy.x() * 0.5f + MARGIN;
        const float half_y = obs.size_xy.y() * 0.5f + MARGIN;
        if (std::abs(point.x() - obs.center_xy.x()) <= half_x &&
            std::abs(point.y() - obs.center_xy.y()) <= half_y &&
            point.z() >= (obs.min_z - MARGIN) && point.z() <= (obs.max_z + MARGIN)) {
            return true;
        }
    }
    return false;
}

bool ClusterExtractor::isInsideOpponentCourt(const Eigen::Vector3f& point, float our_robot_y) const
{
    const float divider = params_.court_y_divider;
    const float margin  = params_.center_barrier_margin;

    if (params_.opponent_side == "side_a") {
        // Opponent is in Area A (positive Y)
        return point.y() > (divider + margin);
    } else if (params_.opponent_side == "side_b") {
        // Opponent is in Area B (negative Y)
        return point.y() < (divider - margin);
    } else {
        // "auto" mode: opponent is on the opposite side of our robot
        if (our_robot_y > divider) {
            // We are in Area A -> Opponent is in Area B
            return point.y() < (divider - margin);
        } else {
            // We are in Area B -> Opponent is in Area A
            return point.y() > (divider + margin);
        }
    }
}

std::vector<Cluster> ClusterExtractor::extractClusters(
    const std::vector<Point3D>& points_in_map, float our_robot_y
)
{
    std::vector<Cluster> valid_clusters;
    if (points_in_map.empty()) {
        return valid_clusters;
    }

    // 1. Filter out points that are outside field, outside opponent court, or inside static
    // obstacles
    std::vector<Point3D> dynamic_filtered_points;
    dynamic_filtered_points.reserve(points_in_map.size());

    for (const auto& pt : points_in_map) {
        const Eigen::Vector3f p(pt.x, pt.y, pt.z);
        if (isInsideOpponentCourt(p, our_robot_y) && !isInsideStaticObstacle(p)) {
            dynamic_filtered_points.push_back(pt);
        }
    }

    if (dynamic_filtered_points.size() < static_cast<size_t>(params_.min_cluster_size)) {
        return valid_clusters;
    }

    // 2. Spatial Grid Hashing (Fast Euclidean Clustering in O(N))
    const float inv_voxel = 1.0f / params_.voxel_size;
    std::unordered_map<int64_t, std::vector<int>> grid_map;
    grid_map.reserve(dynamic_filtered_points.size());

    for (size_t i = 0; i < dynamic_filtered_points.size(); ++i) {
        const auto& pt     = dynamic_filtered_points[i];
        const int gx       = static_cast<int>(std::floor(pt.x * inv_voxel));
        const int gy       = static_cast<int>(std::floor(pt.y * inv_voxel));
        const int gz       = static_cast<int>(std::floor(pt.z * inv_voxel));
        const int64_t hash = computeGridHash(gx, gy, gz);
        grid_map[hash].push_back(static_cast<int>(i));
    }

    // 3. BFS / Connected Component Grouping
    std::vector<bool> visited(dynamic_filtered_points.size(), false);
    const int neighbor_range = static_cast<int>(std::ceil(params_.cluster_tolerance * inv_voxel));

    for (size_t i = 0; i < dynamic_filtered_points.size(); ++i) {
        if (visited[i]) {
            continue;
        }

        std::vector<int> cluster_indices;
        std::queue<int> q;
        q.push(static_cast<int>(i));
        visited[i] = true;

        while (!q.empty()) {
            const int cur_idx = q.front();
            q.pop();
            cluster_indices.push_back(cur_idx);

            const auto& cur_pt = dynamic_filtered_points[cur_idx];
            const int cgx      = static_cast<int>(std::floor(cur_pt.x * inv_voxel));
            const int cgy      = static_cast<int>(std::floor(cur_pt.y * inv_voxel));
            const int cgz      = static_cast<int>(std::floor(cur_pt.z * inv_voxel));

            // Search neighboring voxels
            for (int dx = -neighbor_range; dx <= neighbor_range; ++dx) {
                for (int dy = -neighbor_range; dy <= neighbor_range; ++dy) {
                    for (int dz = -neighbor_range; dz <= neighbor_range; ++dz) {
                        const int64_t n_hash = computeGridHash(cgx + dx, cgy + dy, cgz + dz);
                        auto it              = grid_map.find(n_hash);
                        if (it == grid_map.end()) {
                            continue;
                        }

                        for (const int n_idx : it->second) {
                            if (visited[n_idx]) {
                                continue;
                            }

                            const auto& n_pt    = dynamic_filtered_points[n_idx];
                            const float dist_sq = (cur_pt.x - n_pt.x) * (cur_pt.x - n_pt.x) +
                                                  (cur_pt.y - n_pt.y) * (cur_pt.y - n_pt.y) +
                                                  (cur_pt.z - n_pt.z) * (cur_pt.z - n_pt.z);

                            if (dist_sq <= params_.cluster_tolerance * params_.cluster_tolerance) {
                                visited[n_idx] = true;
                                q.push(n_idx);
                            }
                        }
                    }
                }
            }
        }

        // Size threshold filter
        if (static_cast<int>(cluster_indices.size()) < params_.min_cluster_size ||
            static_cast<int>(cluster_indices.size()) > params_.max_cluster_size) {
            continue;
        }

        // Build Cluster object
        Cluster cluster;
        cluster.points.reserve(cluster_indices.size());
        Eigen::Vector3f min_b(
            std::numeric_limits<float>::max(),
            std::numeric_limits<float>::max(),
            std::numeric_limits<float>::max()
        );
        Eigen::Vector3f max_b(
            std::numeric_limits<float>::lowest(),
            std::numeric_limits<float>::lowest(),
            std::numeric_limits<float>::lowest()
        );
        Eigen::Vector3f sum_pts = Eigen::Vector3f::Zero();

        for (const int idx : cluster_indices) {
            const auto& pt = dynamic_filtered_points[idx];
            cluster.points.push_back(pt);
            sum_pts += Eigen::Vector3f(pt.x, pt.y, pt.z);

            min_b.x() = std::min(min_b.x(), pt.x);
            min_b.y() = std::min(min_b.y(), pt.y);
            min_b.z() = std::min(min_b.z(), pt.z);

            max_b.x() = std::max(max_b.x(), pt.x);
            max_b.y() = std::max(max_b.y(), pt.y);
            max_b.z() = std::max(max_b.z(), pt.z);
        }

        cluster.bbox.min_bound  = min_b;
        cluster.bbox.max_bound  = max_b;
        cluster.bbox.center     = 0.5f * (min_b + max_b);
        cluster.bbox.dimensions = max_b - min_b;
        cluster.centroid        = sum_pts / static_cast<float>(cluster.points.size());

        // 4. Validate geometric constraints for Robocon 2026 Opponent Robot
        if (isValidOpponentRobot(cluster)) {
            extractBucketCenter(cluster);
            valid_clusters.push_back(cluster);
        }
    }

    return valid_clusters;
}

bool ClusterExtractor::isValidOpponentRobot(const Cluster& cluster) const
{
    const auto& dim = cluster.bbox.dimensions;

    // Horizontal dimensions check (Rule 3.2.2: max 1200mm x 1200mm)
    if (dim.x() < params_.robot_min_width || dim.x() > params_.robot_max_width) {
        return false;
    }
    if (dim.y() < params_.robot_min_depth || dim.y() > params_.robot_max_depth) {
        return false;
    }

    // Height check
    if (dim.z() < params_.robot_min_height || dim.z() > params_.robot_max_height) {
        return false;
    }

    // Floor grounding check (must touch floor, Z_min <= 0.25m in map frame)
    if (cluster.bbox.min_bound.z() > params_.robot_max_ground_z) {
        return false;  // Exclude flying objects (towels, overhead cameras)
    }

    return true;
}

bool ClusterExtractor::extractBucketCenter(Cluster& cluster) const
{
    // Extract points located in the mobile bucket height range [1.15m, 2.15m]
    Eigen::Vector3f bucket_sum = Eigen::Vector3f::Zero();
    int bucket_count           = 0;

    for (const auto& pt : cluster.points) {
        if (pt.z >= params_.bucket_min_z && pt.z <= params_.bucket_max_z) {
            bucket_sum += Eigen::Vector3f(pt.x, pt.y, pt.z);
            ++bucket_count;
        }
    }

    if (bucket_count >= 5) {
        cluster.bucket_center = bucket_sum / static_cast<float>(bucket_count);
        cluster.has_bucket    = true;
        cluster.confidence    = 1.0f;
        return true;
    }

    // Fallback: Use the top centroid of the bounding box
    cluster.bucket_center = Eigen::Vector3f(
        cluster.centroid.x(),
        cluster.centroid.y(),
        std::max(cluster.bbox.max_bound.z() - 0.15f, params_.bucket_min_z)
    );
    cluster.has_bucket = false;
    cluster.confidence = 0.7f;
    return false;
}

}  // namespace opponent_tracker
