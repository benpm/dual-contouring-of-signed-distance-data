#pragma once

#include <Eigen/Core>

#include <functional>
#include <vector>

struct ClosestPointQuery {
    double squared_distance = 0.0;
    int triangle_index = -1;
    int face_index = -1;
    Eigen::Vector3d point = Eigen::Vector3d::Zero();
};

class TriangleBvh {
public:
    TriangleBvh() = default;
    TriangleBvh(const Eigen::MatrixXd& vertices, const Eigen::MatrixXi& faces);

    void build(const Eigen::MatrixXd& vertices, const Eigen::MatrixXi& faces);
    bool empty() const;
    ClosestPointQuery closest_point(const Eigen::Vector3d& point) const;
    void closest_points(
        const Eigen::MatrixXd& points,
        Eigen::VectorXd& squared_distances,
        Eigen::VectorXi& triangle_indices,
        Eigen::MatrixXd& closest_points
    ) const;

private:
    struct Triangle {
        Eigen::Vector3d a;
        Eigen::Vector3d b;
        Eigen::Vector3d c;
        Eigen::Vector3d minimum;
        Eigen::Vector3d maximum;
        Eigen::Vector3d centroid;
        int face_index = -1;
    };

    struct Node {
        Eigen::Vector3d minimum;
        Eigen::Vector3d maximum;
        int first = 0;
        int count = 0;
        int left = -1;
        int right = -1;
    };

    int build_node(int first, int count);
    void query_node(int node_index, const Eigen::Vector3d& point, ClosestPointQuery& result) const;

    std::vector<Triangle> triangles_;
    std::vector<int> indices_;
    std::vector<Node> nodes_;
};

bool marching_cubes(
    const Eigen::VectorXd& samples,
    const Eigen::MatrixXd& positions,
    int resolution_x,
    int resolution_y,
    int resolution_z,
    double iso_value,
    Eigen::MatrixXd& vertices,
    Eigen::MatrixXi& faces,
    const std::function<bool()>& cancel_requested = {}
);
