#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace opponent_tracker {

struct Point3D {
    float x;
    float y;
    float z;
    float intensity{0.0f};
};

struct BoundingBox3D {
    Eigen::Vector3f min_bound;
    Eigen::Vector3f max_bound;
    Eigen::Vector3f center;
    Eigen::Vector3f dimensions;
};

struct StaticObstacle {
    std::string name;
    Eigen::Vector2f center_xy;
    Eigen::Vector2f size_xy;
    float min_z;
    float max_z;
};

struct Cluster {
    std::vector<Point3D> points;
    BoundingBox3D bbox;
    Eigen::Vector3f centroid;
    Eigen::Vector3f bucket_center;  // Aiming point for cloth shooter (Z in [1.2m, 2.1m])
    bool has_bucket{false};
    float confidence{0.0f};
};

struct ClusterParams {
    float voxel_size{0.08f};         // Grid voxel size for spatial indexing
    float cluster_tolerance{0.20f};  // Max distance between neighboring voxels/points
    int min_cluster_size{15};        // Min points per cluster
    int max_cluster_size{10000};     // Max points per cluster

    // Robocon 2026 rule constraints for Opponent Robot
    float robot_min_width{0.20f};   // Min bounding box width [m]
    float robot_max_width{1.30f};   // Max bounding box width [m] (Rule 3.2.2: max 1200mm)
    float robot_min_depth{0.20f};   // Min bounding box depth [m]
    float robot_max_depth{1.30f};   // Max bounding box depth [m] (Rule 3.2.2: max 1200mm)
    float robot_min_height{0.30f};  // Min bounding box height [m]
    float robot_max_height{2.20f};  // Max bounding box height [m] (Rule 3.2.2: max 1800mm + bucket)
    float robot_max_ground_z{0.25f};  // Robot must touch floor (Z_min in map frame <= 0.25m)

    // Mobile Bucket parameters (PO-24A bucket: phi 273mm, H 255mm)
    float bucket_min_z{1.15f};   // Min height of mobile bucket [m]
    float bucket_max_z{2.15f};   // Max height of mobile bucket [m] (Rule 3.2.3: 1200-2100mm)
    float bucket_radius{0.14f};  // Radius of PO-24A bucket (~136.5mm)

    // Field boundary in map frame [m]
    float field_min_x{-6.0f};
    float field_max_x{6.0f};
    float field_min_y{-6.5f};
    float field_max_y{6.5f};

    // Court filtering parameters
    // "auto": Auto-detect based on our robot's Y position
    // "side_a": Area A (Y > court_y_divider + center_barrier_margin)
    // "side_b": Area B (Y < court_y_divider - center_barrier_margin)
    std::string opponent_side{"auto"};
    float court_y_divider{0.0f};         // Center line Y [m]
    float center_barrier_margin{0.30f};  // Margin around center divider [m]
};

class ClusterExtractor
{
public:
    explicit ClusterExtractor(const ClusterParams& params = ClusterParams());

    void setParams(const ClusterParams& params);
    const ClusterParams& getParams() const;

    // Set predefined static obstacles (B1, B2, B3, desks, flag, podium)
    void setStaticObstacles(const std::vector<StaticObstacle>& obstacles);

    // Extract clusters from points (in map frame) and classify the opponent robot
    std::vector<Cluster> extractClusters(
        const std::vector<Point3D>& points_in_map, float our_robot_y = 0.0f
    );

    // Check if a point is within the valid opponent court
    bool isInsideOpponentCourt(const Eigen::Vector3f& point, float our_robot_y) const;

    // Filter out static obstacles
    bool isInsideStaticObstacle(const Eigen::Vector3f& point) const;

    // Evaluate whether a cluster matches the opponent robot geometry
    bool isValidOpponentRobot(const Cluster& cluster) const;

    // Extract top bucket center from a valid robot cluster
    bool extractBucketCenter(Cluster& cluster) const;

private:
    ClusterParams params_;
    std::vector<StaticObstacle> static_obstacles_;

    // Spatial hash key calculation
    inline int64_t computeGridHash(int gx, int gy, int gz) const
    {
        return (static_cast<int64_t>(gx) * 73856093) ^ (static_cast<int64_t>(gy) * 19349663) ^
               (static_cast<int64_t>(gz) * 83492791);
    }
};

}  // namespace opponent_tracker
