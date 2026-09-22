#include "geometry.h"

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <unordered_map>

namespace {

#include "marching_cubes_tables.inc"

double squared_distance_to_box(
    const Eigen::Vector3d& point,
    const Eigen::Vector3d& minimum,
    const Eigen::Vector3d& maximum
) {
    double result = 0.0;
    for (int axis = 0; axis < 3; ++axis) {
        if (point[axis] < minimum[axis]) {
            const double difference = minimum[axis] - point[axis];
            result += difference * difference;
        } else if (point[axis] > maximum[axis]) {
            const double difference = point[axis] - maximum[axis];
            result += difference * difference;
        }
    }
    return result;
}

Eigen::Vector3d closest_point_on_segment(
    const Eigen::Vector3d& point,
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& end
) {
    const Eigen::Vector3d direction = end - start;
    const double length_squared = direction.squaredNorm();
    if (length_squared <= 1e-24) return start;
    const double parameter = std::clamp((point - start).dot(direction) / length_squared, 0.0, 1.0);
    return start + parameter * direction;
}

Eigen::Vector3d closest_point_on_triangle(
    const Eigen::Vector3d& point,
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c
) {
    const Eigen::Vector3d ab = b - a;
    const Eigen::Vector3d ac = c - a;
    if (ab.cross(ac).squaredNorm() <= 1e-24) {
        const Eigen::Vector3d on_ab = closest_point_on_segment(point, a, b);
        const Eigen::Vector3d on_bc = closest_point_on_segment(point, b, c);
        const Eigen::Vector3d on_ca = closest_point_on_segment(point, c, a);
        const double ab_distance = (point - on_ab).squaredNorm();
        const double bc_distance = (point - on_bc).squaredNorm();
        const double ca_distance = (point - on_ca).squaredNorm();
        if (ab_distance <= bc_distance && ab_distance <= ca_distance) return on_ab;
        return bc_distance <= ca_distance ? on_bc : on_ca;
    }

    const Eigen::Vector3d ap = point - a;
    const double d1 = ab.dot(ap);
    const double d2 = ac.dot(ap);
    if (d1 <= 0.0 && d2 <= 0.0) return a;

    const Eigen::Vector3d bp = point - b;
    const double d3 = ab.dot(bp);
    const double d4 = ac.dot(bp);
    if (d3 >= 0.0 && d4 <= d3) return b;

    const double vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
        return a + (d1 / (d1 - d3)) * ab;
    }

    const Eigen::Vector3d cp = point - c;
    const double d5 = ab.dot(cp);
    const double d6 = ac.dot(cp);
    if (d6 >= 0.0 && d5 <= d6) return c;

    const double vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
        return a + (d2 / (d2 - d6)) * ac;
    }

    const double va = d3 * d6 - d5 * d4;
    if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
        return b + ((d4 - d3) / ((d4 - d3) + (d5 - d6))) * (c - b);
    }

    const double denominator = 1.0 / (va + vb + vc);
    return a + ab * (vb * denominator) + ac * (vc * denominator);
}

std::int64_t edge_key(int first, int second) {
    if (first > second) std::swap(first, second);
    return static_cast<std::int64_t>(static_cast<std::uint32_t>(first)) |
        (static_cast<std::int64_t>(static_cast<std::uint32_t>(second)) << 32);
}

} // namespace

TriangleBvh::TriangleBvh(const Eigen::MatrixXd& vertices, const Eigen::MatrixXi& faces) {
    build(vertices, faces);
}

void TriangleBvh::build(const Eigen::MatrixXd& vertices, const Eigen::MatrixXi& faces) {
    triangles_.clear();
    indices_.clear();
    nodes_.clear();
    if (vertices.cols() != 3 || faces.rows() == 0 || (faces.cols() != 3 && faces.cols() != 4)) return;

    const int triangles_per_face = faces.cols() == 4 ? 2 : 1;
    triangles_.reserve(static_cast<std::size_t>(faces.rows() * triangles_per_face));
    for (int face = 0; face < faces.rows(); ++face) {
        const std::array<std::array<int, 3>, 2> corners{{
            {{faces(face, 0), faces(face, 1), faces(face, 2)}},
            {{faces(face, 0), faces(face, 2), faces(face, faces.cols() - 1)}}
        }};
        for (int triangle = 0; triangle < triangles_per_face; ++triangle) {
            const auto& indices = corners[triangle];
            if (indices[0] < 0 || indices[1] < 0 || indices[2] < 0 ||
                indices[0] >= vertices.rows() || indices[1] >= vertices.rows() || indices[2] >= vertices.rows()) {
                continue;
            }
            Triangle primitive;
            primitive.a = vertices.row(indices[0]);
            primitive.b = vertices.row(indices[1]);
            primitive.c = vertices.row(indices[2]);
            primitive.minimum = primitive.a.cwiseMin(primitive.b).cwiseMin(primitive.c);
            primitive.maximum = primitive.a.cwiseMax(primitive.b).cwiseMax(primitive.c);
            primitive.centroid = (primitive.a + primitive.b + primitive.c) / 3.0;
            primitive.face_index = face;
            triangles_.push_back(primitive);
        }
    }
    indices_.resize(triangles_.size());
    std::iota(indices_.begin(), indices_.end(), 0);
    if (!indices_.empty()) build_node(0, static_cast<int>(indices_.size()));
}

bool TriangleBvh::empty() const {
    return nodes_.empty();
}

int TriangleBvh::build_node(int first, int count) {
    Node node;
    node.minimum = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
    node.maximum = Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());
    Eigen::Vector3d centroid_minimum = node.minimum;
    Eigen::Vector3d centroid_maximum = node.maximum;
    for (int offset = 0; offset < count; ++offset) {
        const Triangle& triangle = triangles_[indices_[first + offset]];
        node.minimum = node.minimum.cwiseMin(triangle.minimum);
        node.maximum = node.maximum.cwiseMax(triangle.maximum);
        centroid_minimum = centroid_minimum.cwiseMin(triangle.centroid);
        centroid_maximum = centroid_maximum.cwiseMax(triangle.centroid);
    }

    const int node_index = static_cast<int>(nodes_.size());
    nodes_.push_back(node);
    if (count <= 8) {
        nodes_[node_index].first = first;
        nodes_[node_index].count = count;
        return node_index;
    }

    Eigen::Index axis = 0;
    (centroid_maximum - centroid_minimum).maxCoeff(&axis);
    const int middle = first + count / 2;
    std::nth_element(
        indices_.begin() + first,
        indices_.begin() + middle,
        indices_.begin() + first + count,
        [this, axis](int left, int right) {
            return triangles_[left].centroid[axis] < triangles_[right].centroid[axis];
        }
    );
    nodes_[node_index].left = build_node(first, middle - first);
    nodes_[node_index].right = build_node(middle, first + count - middle);
    return node_index;
}

ClosestPointQuery TriangleBvh::closest_point(const Eigen::Vector3d& point) const {
    ClosestPointQuery result;
    result.squared_distance = std::numeric_limits<double>::infinity();
    if (!nodes_.empty()) query_node(0, point, result);
    return result;
}

void TriangleBvh::query_node(
    int node_index,
    const Eigen::Vector3d& point,
    ClosestPointQuery& result
) const {
    const Node& node = nodes_[node_index];
    if (squared_distance_to_box(point, node.minimum, node.maximum) > result.squared_distance) return;
    if (node.count > 0) {
        for (int offset = 0; offset < node.count; ++offset) {
            const int triangle_index = indices_[node.first + offset];
            const Triangle& triangle = triangles_[triangle_index];
            const Eigen::Vector3d closest = closest_point_on_triangle(point, triangle.a, triangle.b, triangle.c);
            const double squared_distance = (point - closest).squaredNorm();
            if (squared_distance < result.squared_distance) {
                result.squared_distance = squared_distance;
                result.triangle_index = triangle_index;
                result.face_index = triangle.face_index;
                result.point = closest;
            }
        }
        return;
    }

    const double left_distance = squared_distance_to_box(
        point, nodes_[node.left].minimum, nodes_[node.left].maximum);
    const double right_distance = squared_distance_to_box(
        point, nodes_[node.right].minimum, nodes_[node.right].maximum);
    if (left_distance <= right_distance) {
        query_node(node.left, point, result);
        query_node(node.right, point, result);
    } else {
        query_node(node.right, point, result);
        query_node(node.left, point, result);
    }
}

void TriangleBvh::closest_points(
    const Eigen::MatrixXd& points,
    Eigen::VectorXd& squared_distances,
    Eigen::VectorXi& triangle_indices,
    Eigen::MatrixXd& closest_points_output
) const {
    squared_distances.resize(points.rows());
    triangle_indices.resize(points.rows());
    closest_points_output.resize(points.rows(), 3);
    for (int row = 0; row < points.rows(); ++row) {
        const ClosestPointQuery result = closest_point(points.row(row));
        squared_distances[row] = result.squared_distance;
        triangle_indices[row] = result.triangle_index;
        closest_points_output.row(row) = result.point;
    }
}

bool marching_cubes(
    const Eigen::VectorXd& samples,
    const Eigen::MatrixXd& positions,
    int resolution_x,
    int resolution_y,
    int resolution_z,
    double iso_value,
    Eigen::MatrixXd& vertices,
    Eigen::MatrixXi& faces,
    const std::function<bool()>& cancel_requested
) {
    const int corner_offsets[8] = {
        0,
        1,
        1 + resolution_x,
        resolution_x,
        resolution_x * resolution_y,
        1 + resolution_x * resolution_y,
        1 + resolution_x + resolution_x * resolution_y,
        resolution_x + resolution_x * resolution_y
    };
    std::vector<Eigen::Vector3d> output_vertices;
    std::vector<Eigen::Vector3i> output_faces;
    std::unordered_map<std::int64_t, int> edge_vertices;

    const auto grid_index = [resolution_x, resolution_y](int x, int y, int z) {
        return x + resolution_x * (y + resolution_y * z);
    };

    for (int z = 0; z < resolution_z - 1; ++z) {
        if (cancel_requested && cancel_requested()) return false;
        for (int y = 0; y < resolution_y - 1; ++y) {
            for (int x = 0; x < resolution_x - 1; ++x) {
                const int base = grid_index(x, y, z);
                std::array<int, 8> indices{};
                std::array<double, 8> values{};
                int cube_flags = 0;
                for (int corner = 0; corner < 8; ++corner) {
                    indices[corner] = base + corner_offsets[corner];
                    values[corner] = samples[indices[corner]];
                    if (values[corner] > iso_value) cube_flags |= 1 << corner;
                }
                const int edge_flags = aiCubeEdgeFlags[cube_flags];
                if (edge_flags == 0) continue;

                std::array<int, 12> local_vertices{};
                local_vertices.fill(-1);
                for (int edge = 0; edge < 12; ++edge) {
                    if ((edge_flags & (1 << edge)) == 0) continue;
                    const int first_corner = a2eConnection[edge][0];
                    const int second_corner = a2eConnection[edge][1];
                    const int first_index = indices[first_corner];
                    const int second_index = indices[second_corner];
                    const std::int64_t key = edge_key(first_index, second_index);
                    const auto found = edge_vertices.find(key);
                    if (found != edge_vertices.end()) {
                        local_vertices[edge] = found->second;
                        continue;
                    }

                    const double first_value = values[first_corner];
                    const double second_value = values[second_corner];
                    const double difference = second_value - first_value;
                    const double interpolation = std::abs(difference) <= 1e-15
                        ? 0.5
                        : std::clamp((iso_value - first_value) / difference, 0.0, 1.0);
                    const Eigen::Vector3d first_position = positions.row(first_index);
                    const Eigen::Vector3d second_position = positions.row(second_index);
                    const int vertex_index = static_cast<int>(output_vertices.size());
                    output_vertices.push_back(first_position + interpolation * (second_position - first_position));
                    edge_vertices.emplace(key, vertex_index);
                    local_vertices[edge] = vertex_index;
                }

                for (int offset = 0; offset < 15; offset += 3) {
                    if (a2fConnectionTable[cube_flags][offset] < 0) break;
                    output_faces.emplace_back(
                        local_vertices[a2fConnectionTable[cube_flags][offset]],
                        local_vertices[a2fConnectionTable[cube_flags][offset + 1]],
                        local_vertices[a2fConnectionTable[cube_flags][offset + 2]]
                    );
                }
            }
        }
    }

    vertices.resize(static_cast<int>(output_vertices.size()), 3);
    for (int row = 0; row < vertices.rows(); ++row) vertices.row(row) = output_vertices[row];
    faces.resize(static_cast<int>(output_faces.size()), 3);
    for (int row = 0; row < faces.rows(); ++row) faces.row(row) = output_faces[row];
    return true;
}
