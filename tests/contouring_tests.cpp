#include "contouring.h"

#include <Eigen/Core>

#include <cassert>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace {

struct GridFixture {
    int resolution = 10;
    Eigen::MatrixXd positions;
    Eigen::VectorXd samples;
};

GridFixture make_sphere_grid() {
    GridFixture fixture;
    const int n = fixture.resolution;
    fixture.positions.resize(n * n * n, 3);
    fixture.samples.resize(n * n * n);
    for (int k = 0; k < n; ++k) {
        for (int j = 0; j < n; ++j) {
            for (int i = 0; i < n; ++i) {
                const int index = i + n * (j + n * k);
                const Eigen::Vector3d point(
                    -1.0 + 2.0 * i / (n - 1.0),
                    -1.0 + 2.0 * j / (n - 1.0),
                    -1.0 + 2.0 * k / (n - 1.0)
                );
                fixture.positions.row(index) = point.transpose();
                fixture.samples[index] = point.norm() - 0.6;
            }
        }
    }
    return fixture;
}

void test_methods_generate_meshes() {
    const GridFixture fixture = make_sphere_grid();
    for (ContouringMethod method : {
        ContouringMethod::MarchingCubes,
        ContouringMethod::DualContouring,
        ContouringMethod::Ours
    }) {
        ContouringOptions options;
        options.method = method;
        options.outer_iters = 0;
        options.inner_iters = 1;
        Eigen::MatrixXd vertices;
        Eigen::MatrixXi faces;
        contouring(
            fixture.samples,
            fixture.positions,
            fixture.resolution,
            fixture.resolution,
            fixture.resolution,
            0.0,
            vertices,
            faces,
            options
        );
        assert(vertices.rows() > 0);
        assert(faces.rows() > 0);
        assert(vertices.allFinite());
    }
}

void test_cancellation_preserves_outputs() {
    const GridFixture fixture = make_sphere_grid();
    ContouringOptions options;
    Eigen::MatrixXd vertices = Eigen::MatrixXd::Constant(1, 3, 42.0);
    Eigen::MatrixXi faces = Eigen::MatrixXi::Constant(1, 3, 7);
    ContouringCallbacks callbacks;
    callbacks.cancel_requested = []() { return true; };
    const ContouringStatus status = contouring(
        fixture.samples,
        fixture.positions,
        fixture.resolution,
        fixture.resolution,
        fixture.resolution,
        0.0,
        vertices,
        faces,
        options,
        callbacks
    );
    assert(status == ContouringStatus::Cancelled);
    assert(vertices.rows() == 1 && vertices(0, 0) == 42.0);
    assert(faces.rows() == 1 && faces(0, 0) == 7);
}

void test_progress_completes() {
    const GridFixture fixture = make_sphere_grid();
    ContouringOptions options;
    options.method = ContouringMethod::MarchingCubes;
    std::vector<ContouringStage> stages;
    ContouringCallbacks callbacks;
    callbacks.progress = [&stages](const ContouringProgress& progress) {
        assert(progress.fraction >= 0.0 && progress.fraction <= 1.0);
        stages.push_back(progress.stage);
    };
    Eigen::MatrixXd vertices;
    Eigen::MatrixXi faces;
    const ContouringStatus status = contouring(
        fixture.samples,
        fixture.positions,
        fixture.resolution,
        fixture.resolution,
        fixture.resolution,
        0.0,
        vertices,
        faces,
        options,
        callbacks
    );
    assert(status == ContouringStatus::Completed);
    assert(!stages.empty());
    assert(stages.back() == ContouringStage::Complete);
}

void test_validation() {
    const GridFixture fixture = make_sphere_grid();
    ContouringOptions options;
    bool threw = false;
    try {
        validate_contouring_input(
            fixture.samples,
            fixture.positions,
            1,
            fixture.resolution,
            fixture.resolution,
            0.0,
            options
        );
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    assert(threw);
}

} // namespace

int main() {
    test_methods_generate_meshes();
    test_cancellation_preserves_outputs();
    test_progress_completes();
    test_validation();
    return 0;
}
