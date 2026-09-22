#include <dcsdd/contouring.h>

#include "contouring.h"

namespace {

ContouringMethod to_internal_method(dcsdd::Method method) {
    switch (method) {
    case dcsdd::Method::MarchingCubes:
        return ContouringMethod::MarchingCubes;
    case dcsdd::Method::DualContouring:
        return ContouringMethod::DualContouring;
    case dcsdd::Method::Optimized:
        return ContouringMethod::Ours;
    }
    return ContouringMethod::DualContouring;
}

dcsdd::Stage to_public_stage(ContouringStage stage) {
    switch (stage) {
    case ContouringStage::Validating:
    case ContouringStage::SamplingSdf:
        return dcsdd::Stage::Validating;
    case ContouringStage::GeneratingCells:
        return dcsdd::Stage::GeneratingCells;
    case ContouringStage::SolvingQefs:
        return dcsdd::Stage::SolvingQefs;
    case ContouringStage::AssigningSpheres:
        return dcsdd::Stage::AssigningSpheres;
    case ContouringStage::ComputingIntersections:
        return dcsdd::Stage::ComputingIntersections;
    case ContouringStage::UpdatingHermiteData:
        return dcsdd::Stage::UpdatingHermiteData;
    case ContouringStage::RefiningVertices:
        return dcsdd::Stage::RefiningVertices;
    case ContouringStage::ExtractingMesh:
        return dcsdd::Stage::ExtractingMesh;
    case ContouringStage::Complete:
        return dcsdd::Stage::Complete;
    }
    return dcsdd::Stage::Validating;
}

} // namespace

namespace dcsdd {

Status generate(
    const SampleGrid& grid,
    Mesh& output,
    const Options& options,
    const Callbacks& callbacks
) {
    ContouringOptions internal_options;
    internal_options.method = to_internal_method(options.method);
    internal_options.verbose = false;
    internal_options.mu = options.regularization_weight;
    internal_options.dc_weight = options.dual_contouring_weight;
    internal_options.sphere_weight = options.sphere_weight;
    internal_options.svd_threshold = options.svd_threshold;
    internal_options.outer_iters = options.outer_iterations;
    internal_options.inner_iters = options.inner_iterations;
    internal_options.hermite_update = options.update_hermite_data;
    internal_options.new_hermite_pos_weight = options.hermite_position_blend;
    internal_options.new_face_pos_weight = options.face_position_blend;
    internal_options.new_hermite_normal_weight = options.hermite_normal_blend;
    internal_options.batch_size = 200000;

    ContouringCallbacks internal_callbacks;
    internal_callbacks.cancel_requested = callbacks.cancel_requested;
    if (callbacks.progress) {
        internal_callbacks.progress = [&callbacks](const ContouringProgress& progress) {
            callbacks.progress({to_public_stage(progress.stage), progress.fraction, progress.message});
        };
    }

    Eigen::MatrixXd vertices;
    Eigen::MatrixXi faces;
    const ContouringStatus status = contouring(
        grid.samples,
        grid.positions,
        grid.dimensions.x(),
        grid.dimensions.y(),
        grid.dimensions.z(),
        grid.iso_value,
        vertices,
        faces,
        internal_options,
        internal_callbacks
    );
    if (status == ContouringStatus::Cancelled) return Status::Cancelled;
    output.vertices = std::move(vertices);
    output.faces = std::move(faces);
    return Status::Completed;
}

} // namespace dcsdd
