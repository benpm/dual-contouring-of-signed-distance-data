#include "analytic_sdf.h"

#include <algorithm>
#include <cmath>

namespace dcsdd::analytic {

double octahedron(const Eigen::Vector3d& point, double size) {
    const Eigen::Vector3d absolute_point = point.cwiseAbs();
    const double plane_distance = absolute_point.sum() - size;

    Eigen::Vector3d permuted_point;
    if (3.0 * absolute_point.x() < plane_distance) {
        permuted_point = absolute_point;
    } else if (3.0 * absolute_point.y() < plane_distance) {
        permuted_point << absolute_point.y(), absolute_point.z(), absolute_point.x();
    } else if (3.0 * absolute_point.z() < plane_distance) {
        permuted_point << absolute_point.z(), absolute_point.x(), absolute_point.y();
    } else {
        return plane_distance / std::sqrt(3.0);
    }

    const double edge_parameter = std::clamp(
        0.5 * (permuted_point.z() - permuted_point.y() + size),
        0.0,
        size
    );
    return Eigen::Vector3d(
        permuted_point.x(),
        permuted_point.y() - size + edge_parameter,
        permuted_point.z() - edge_parameter
    ).norm();
}

} // namespace dcsdd::analytic
