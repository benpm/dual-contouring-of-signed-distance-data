#include <dcsdd/contouring.h>

#include "geometry.h"

#include <Eigen/Core>

#include <cassert>
#include <stdexcept>
#include <vector>

namespace {

dcsdd::SampleGrid make_sphere_grid(int resolution = 10) {
    dcsdd::SampleGrid grid;
    grid.dimensions = Eigen::Vector3i::Constant(resolution);
    grid.positions.resize(resolution * resolution * resolution, 3);
    grid.samples.resize(resolution * resolution * resolution);
    for (int z = 0; z < resolution; ++z) {
        for (int y = 0; y < resolution; ++y) {
            for (int x = 0; x < resolution; ++x) {
                const int index = x + resolution * (y + resolution * z);
                const Eigen::Vector3d point(
                    -1.0 + 2.0 * x / (resolution - 1.0),
                    -1.0 + 2.0 * y / (resolution - 1.0),
                    -1.0 + 2.0 * z / (resolution - 1.0)
                );
                grid.positions.row(index) = point;
                grid.samples[index] = point.norm() - 0.6;
            }
        }
    }
    return grid;
}

void test_methods_generate_meshes() {
    const dcsdd::SampleGrid grid = make_sphere_grid();
    for (dcsdd::Method method : {
        dcsdd::Method::MarchingCubes,
        dcsdd::Method::DualContouring,
        dcsdd::Method::Optimized
    }) {
        dcsdd::Options options;
        options.method = method;
        options.outer_iterations = 0;
        options.inner_iterations = 1;
        dcsdd::Mesh mesh;
        assert(dcsdd::generate(grid, mesh, options) == dcsdd::Status::Completed);
        assert(mesh.vertices.rows() > 0);
        assert(mesh.faces.rows() > 0);
        assert(mesh.vertices.allFinite());
    }
}

void test_cancellation_preserves_output() {
    const dcsdd::SampleGrid grid = make_sphere_grid();
    dcsdd::Mesh mesh;
    mesh.vertices = Eigen::MatrixXd::Constant(1, 3, 42.0);
    mesh.faces = Eigen::MatrixXi::Constant(1, 3, 7);
    dcsdd::Callbacks callbacks;
    callbacks.cancel_requested = []() { return true; };
    assert(dcsdd::generate(grid, mesh, {}, callbacks) == dcsdd::Status::Cancelled);
    assert(mesh.vertices.rows() == 1 && mesh.vertices(0, 0) == 42.0);
    assert(mesh.faces.rows() == 1 && mesh.faces(0, 0) == 7);
}

void test_progress_completes() {
    const dcsdd::SampleGrid grid = make_sphere_grid();
    dcsdd::Options options;
    options.method = dcsdd::Method::MarchingCubes;
    std::vector<dcsdd::Stage> stages;
    dcsdd::Callbacks callbacks;
    callbacks.progress = [&stages](const dcsdd::Progress& progress) {
        assert(progress.fraction >= 0.0 && progress.fraction <= 1.0);
        stages.push_back(progress.stage);
    };
    dcsdd::Mesh mesh;
    assert(dcsdd::generate(grid, mesh, options, callbacks) == dcsdd::Status::Completed);
    assert(!stages.empty());
    assert(stages.back() == dcsdd::Stage::Complete);
}

void test_validation() {
    dcsdd::SampleGrid grid = make_sphere_grid();
    grid.dimensions.x() = 1;
    dcsdd::Mesh mesh;
    bool threw = false;
    try {
        dcsdd::generate(grid, mesh);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    assert(threw);
}

void test_triangle_bvh() {
    Eigen::MatrixXd vertices(3, 3);
    vertices << 0.0, 0.0, 0.0,
                1.0, 0.0, 0.0,
                0.0, 1.0, 0.0;
    Eigen::MatrixXi faces(1, 3);
    faces << 0, 1, 2;
    const TriangleBvh tree(vertices, faces);
    const ClosestPointQuery query = tree.closest_point(Eigen::Vector3d(0.25, 0.25, 1.0));
    assert(query.face_index == 0);
    assert(std::abs(query.squared_distance - 1.0) < 1e-12);
    assert((query.point - Eigen::Vector3d(0.25, 0.25, 0.0)).norm() < 1e-12);
}

} // namespace

int main() {
    test_methods_generate_meshes();
    test_cancellation_preserves_output();
    test_progress_completes();
    test_validation();
    test_triangle_bvh();
    return 0;
}
