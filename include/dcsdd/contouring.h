#pragma once

#include <Eigen/Core>

#include <functional>
#include <string>

namespace dcsdd {

enum class Method {
    MarchingCubes,
    DualContouring,
    Optimized
};

struct Options {
    Method method = Method::DualContouring;
    double regularization_weight = 0.1;
    double dual_contouring_weight = 0.02;
    double sphere_weight = 1.0;
    double svd_threshold = 0.01;
    int outer_iterations = 100;
    int inner_iterations = 100;
    bool update_hermite_data = true;
    double hermite_position_blend = 0.2;
    double face_position_blend = 0.2;
    double hermite_normal_blend = 0.2;
};

struct SampleGrid {
    Eigen::VectorXd samples;
    Eigen::MatrixXd positions;
    Eigen::Vector3i dimensions = Eigen::Vector3i::Zero();
    double iso_value = 0.0;
};

struct Mesh {
    Eigen::MatrixXd vertices;
    Eigen::MatrixXi faces;
};

enum class Stage {
    Validating,
    GeneratingCells,
    SolvingQefs,
    AssigningSpheres,
    ComputingIntersections,
    UpdatingHermiteData,
    RefiningVertices,
    ExtractingMesh,
    Complete
};

struct Progress {
    Stage stage = Stage::Validating;
    double fraction = 0.0;
    std::string message;
};

struct Callbacks {
    std::function<void(const Progress&)> progress;
    std::function<bool()> cancel_requested;
};

enum class Status {
    Completed,
    Cancelled
};

Status generate(
    const SampleGrid& grid,
    Mesh& output,
    const Options& options = {},
    const Callbacks& callbacks = {}
);

} // namespace dcsdd
