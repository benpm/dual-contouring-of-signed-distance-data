#include "application.h"

#include <imgui.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <exception>
#include <utility>

namespace {

constexpr int kResolution = 33;
constexpr double kBoundsMinimum = -1.1;
constexpr double kBoundsMaximum = 1.1;
constexpr float kPi = 3.14159265358979323846f;

Eigen::Matrix4f perspective(float vertical_field_of_view, float aspect, float near_plane, float far_plane) {
    const float scale = 1.0f / std::tan(vertical_field_of_view * 0.5f);
    Eigen::Matrix4f matrix = Eigen::Matrix4f::Zero();
    matrix(0, 0) = scale / aspect;
    matrix(1, 1) = scale;
    matrix(2, 2) = (far_plane + near_plane) / (near_plane - far_plane);
    matrix(2, 3) = (2.0f * far_plane * near_plane) / (near_plane - far_plane);
    matrix(3, 2) = -1.0f;
    return matrix;
}

Eigen::Matrix4f look_at(
    const Eigen::Vector3f& eye,
    const Eigen::Vector3f& target,
    const Eigen::Vector3f& up
) {
    const Eigen::Vector3f forward = (target - eye).normalized();
    const Eigen::Vector3f right = forward.cross(up).normalized();
    const Eigen::Vector3f corrected_up = right.cross(forward);
    Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
    matrix.block<1, 3>(0, 0) = right.transpose();
    matrix.block<1, 3>(1, 0) = corrected_up.transpose();
    matrix.block<1, 3>(2, 0) = -forward.transpose();
    matrix(0, 3) = -right.dot(eye);
    matrix(1, 3) = -corrected_up.dot(eye);
    matrix(2, 3) = forward.dot(eye);
    return matrix;
}

} // namespace

Application::Application(Renderer& renderer) : renderer_(renderer) {
    examples_ = {
        {1, "Sphere", [](const Eigen::Vector3d& p) { return p.norm() - 0.72; }, {0.92f, 0.32f, 0.38f}, {-2.4f, 1.5f, 0.0f}},
        {2, "Box", [](const Eigen::Vector3d& p) {
            const Eigen::Vector3d q = p.cwiseAbs() - Eigen::Vector3d(0.65, 0.52, 0.44);
            return q.cwiseMax(0.0).norm() + std::min(q.maxCoeff(), 0.0);
        }, {0.27f, 0.68f, 0.95f}, {0.0f, 1.5f, 0.0f}},
        {3, "Torus", [](const Eigen::Vector3d& p) {
            const Eigen::Vector2d q(Eigen::Vector2d(p.x(), p.z()).norm() - 0.55, p.y());
            return q.norm() - 0.22;
        }, {0.98f, 0.68f, 0.22f}, {2.4f, 1.5f, 0.0f}},
        {4, "Cylinder", [](const Eigen::Vector3d& p) {
            const Eigen::Vector2d d(Eigen::Vector2d(p.x(), p.z()).norm() - 0.5, std::abs(p.y()) - 0.6);
            return d.cwiseMax(0.0).norm() + std::min(d.maxCoeff(), 0.0);
        }, {0.35f, 0.82f, 0.52f}, {-2.4f, -1.5f, 0.0f}},
        {5, "Octahedron", [](const Eigen::Vector3d& p) {
            return (p.cwiseAbs().sum() - 0.95) / std::sqrt(3.0);
        }, {0.73f, 0.48f, 0.94f}, {0.0f, -1.5f, 0.0f}},
        {6, "Cut Sphere", [](const Eigen::Vector3d& p) {
            constexpr double radius = 0.75;
            constexpr double height = 0.22;
            const double width = std::sqrt(radius * radius - height * height);
            const Eigen::Vector2d q(Eigen::Vector2d(p.x(), p.z()).norm(), p.y());
            const double selector = std::max(
                (height - radius) * q.x() * q.x() + width * width * (height + radius - 2.0 * q.y()),
                height * q.x() - width * q.y()
            );
            if (selector < 0.0) return q.norm() - radius;
            if (q.x() < width) return height - q.y();
            return (q - Eigen::Vector2d(width, height)).norm();
        }, {0.95f, 0.43f, 0.75f}, {2.4f, -1.5f, 0.0f}}
    };
    rebuild_render_items();
    worker_ = std::thread(&Application::run_worker, this);
}

Application::~Application() {
    cancel_requested_ = true;
    if (worker_.joinable()) worker_.join();
}

dcsdd::SampleGrid Application::sample(const Sdf& sdf) const {
    dcsdd::SampleGrid grid;
    grid.dimensions = Eigen::Vector3i::Constant(kResolution);
    grid.iso_value = 0.0;
    const int sample_count = kResolution * kResolution * kResolution;
    grid.samples.resize(sample_count);
    grid.positions.resize(sample_count, 3);
    for (int z = 0; z < kResolution; ++z) {
        for (int y = 0; y < kResolution; ++y) {
            for (int x = 0; x < kResolution; ++x) {
                const int index = x + kResolution * (y + kResolution * z);
                const Eigen::Vector3d point(
                    kBoundsMinimum + (kBoundsMaximum - kBoundsMinimum) * x / (kResolution - 1.0),
                    kBoundsMinimum + (kBoundsMaximum - kBoundsMinimum) * y / (kResolution - 1.0),
                    kBoundsMinimum + (kBoundsMaximum - kBoundsMinimum) * z / (kResolution - 1.0)
                );
                grid.positions.row(index) = point;
                grid.samples[index] = sdf(point);
            }
        }
    }
    return grid;
}

void Application::run_worker() {
    for (std::size_t index = 0; index < examples_.size() && !cancel_requested_; ++index) {
        WorkerResult result;
        result.example_index = index;
        try {
            dcsdd::Options options;
            options.method = dcsdd::Method::Optimized;
            options.outer_iterations = 5;
            options.inner_iterations = 10;
            const dcsdd::SampleGrid grid = sample(examples_[index].sdf);
            dcsdd::Callbacks callbacks;
            callbacks.cancel_requested = [this]() { return cancel_requested_.load(); };
            callbacks.progress = [this, index](const dcsdd::Progress& progress) {
                std::lock_guard<std::mutex> lock(worker_mutex_);
                progress_fraction_ = (index + progress.fraction) / examples_.size();
                progress_message_ = examples_[index].name + ": " + progress.message;
            };
            if (dcsdd::generate(grid, result.mesh, options, callbacks) == dcsdd::Status::Cancelled) break;
        } catch (const std::exception& exception) {
            result.error = exception.what();
        } catch (...) {
            result.error = "Unknown generation failure";
        }
        std::lock_guard<std::mutex> lock(worker_mutex_);
        pending_results_.push_back(std::move(result));
    }
    {
        std::lock_guard<std::mutex> lock(worker_mutex_);
        progress_fraction_ = 1.0;
        progress_message_ = cancel_requested_ ? "Cancelled" : "Gallery ready";
    }
    worker_finished_ = true;
}

void Application::update() {
    std::deque<WorkerResult> results;
    {
        std::lock_guard<std::mutex> lock(worker_mutex_);
        results.swap(pending_results_);
    }
    for (WorkerResult& result : results) {
        Example& example = examples_[result.example_index];
        example.error = std::move(result.error);
        if (example.error.empty()) {
            renderer_.upload(example.id, result.mesh);
            example.ready = true;
        }
    }
    if (!results.empty()) rebuild_render_items();
}

void Application::rebuild_render_items() {
    render_items_.clear();
    for (const Example& example : examples_) {
        RenderItem item;
        item.id = example.id;
        item.color = example.color;
        item.visible = example.ready && (selected_id_ == 0 || selected_id_ == example.id);
        item.transform = Eigen::Matrix4f::Identity();
        if (selected_id_ == 0) item.transform.block<3, 1>(0, 3) = example.gallery_position;
        else if (selected_id_ == example.id) item.transform.block<3, 3>(0, 0) *= 1.6f;
        render_items_.push_back(item);
    }
}

void Application::draw_ui() {
    ImGui::SetNextWindowPos(ImVec2(16.0f, 16.0f), ImGuiCond_Always);
    ImGui::SetNextWindowBgAlpha(0.88f);
    ImGui::Begin("Gallery", nullptr, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse);
    if (selected_id_ != 0) {
        const Example& selected = examples_[selected_id_ - 1];
        ImGui::TextUnformatted(selected.name.c_str());
        if (ImGui::Button("Back to Gallery")) select(0);
    } else {
        ImGui::TextUnformatted("Click a generated model to inspect it.");
        ImGui::TextDisabled("Right-drag to orbit. Scroll to zoom.");
    }
    std::string message;
    double fraction = 0.0;
    {
        std::lock_guard<std::mutex> lock(worker_mutex_);
        message = progress_message_;
        fraction = progress_fraction_;
    }
    if (!worker_finished_) ImGui::ProgressBar(static_cast<float>(fraction), ImVec2(320.0f, 0.0f), message.c_str());
    else ImGui::TextUnformatted(message.c_str());
    for (const Example& example : examples_) {
        if (!example.error.empty()) ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.3f, 1.0f), "%s: %s", example.name.c_str(), example.error.c_str());
    }
    ImGui::End();
}

void Application::orbit(float horizontal, float vertical) {
    camera_yaw_ -= horizontal * 0.006f;
    camera_pitch_ = std::clamp(camera_pitch_ - vertical * 0.006f, -1.35f, 1.35f);
}

void Application::zoom(float amount) {
    camera_distance_ = std::clamp(camera_distance_ * std::exp(-amount * 0.12f), 2.2f, 18.0f);
}

void Application::select(unsigned int id) {
    if (id > examples_.size() || (id != 0 && !examples_[id - 1].ready)) return;
    selected_id_ = id;
    camera_distance_ = id == 0 ? 9.0f : 3.4f;
    rebuild_render_items();
}

Eigen::Matrix4f Application::view_projection(float aspect_ratio) const {
    const Eigen::Vector3f target = Eigen::Vector3f::Zero();
    const Eigen::Vector3f direction(
        std::cos(camera_pitch_) * std::sin(camera_yaw_),
        std::sin(camera_pitch_),
        std::cos(camera_pitch_) * std::cos(camera_yaw_)
    );
    const Eigen::Vector3f eye = target + camera_distance_ * direction;
    return perspective(45.0f * kPi / 180.0f, std::max(aspect_ratio, 0.1f), 0.05f, 100.0f) *
        look_at(eye, target, Eigen::Vector3f::UnitY());
}

const std::vector<RenderItem>& Application::render_items() const {
    return render_items_;
}
