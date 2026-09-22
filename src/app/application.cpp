#include "application.h"

#include <igl/boundary_facets.h>
#include <igl/is_edge_manifold.h>
#include <igl/read_triangle_mesh.h>
#include <igl/signed_distance.h>
#include <igl/writeOBJ.h>
#include <imgui.h>
#include <polyscope/curve_network.h>
#include <polyscope/polyscope.h>
#include <polyscope/surface_mesh.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <sstream>
#include <stdexcept>

namespace {

constexpr const char* kSourceMeshName = "Source mesh";
constexpr const char* kGeneratedMeshName = "Generated mesh";
constexpr const char* kCropBoxName = "Sampling bounds";

std::string run_command(const char* command) {
    std::array<char, 4096> buffer{};
    std::string output;
    FILE* pipe = popen(command, "r");
    if (!pipe) return {};
    while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe)) {
        output += buffer.data();
    }
    pclose(pipe);
    while (!output.empty() && (output.back() == '\n' || output.back() == '\r')) output.pop_back();
    return output;
}

Eigen::Matrix3d rotation_matrix(const std::array<float, 3>& degrees) {
    constexpr double pi = 3.14159265358979323846;
    const Eigen::AngleAxisd rx(degrees[0] * pi / 180.0, Eigen::Vector3d::UnitX());
    const Eigen::AngleAxisd ry(degrees[1] * pi / 180.0, Eigen::Vector3d::UnitY());
    const Eigen::AngleAxisd rz(degrees[2] * pi / 180.0, Eigen::Vector3d::UnitZ());
    return (rz * ry * rx).toRotationMatrix();
}

Eigen::MatrixXd crop_box_vertices(const Eigen::Vector3d& minimum, const Eigen::Vector3d& maximum) {
    Eigen::MatrixXd vertices(8, 3);
    vertices <<
        minimum.x(), minimum.y(), minimum.z(),
        maximum.x(), minimum.y(), minimum.z(),
        maximum.x(), maximum.y(), minimum.z(),
        minimum.x(), maximum.y(), minimum.z(),
        minimum.x(), minimum.y(), maximum.z(),
        maximum.x(), minimum.y(), maximum.z(),
        maximum.x(), maximum.y(), maximum.z(),
        minimum.x(), maximum.y(), maximum.z();
    return vertices;
}

Eigen::MatrixXi crop_box_edges() {
    Eigen::MatrixXi edges(12, 2);
    edges << 0, 1, 1, 2, 2, 3, 3, 0,
             4, 5, 5, 6, 6, 7, 7, 4,
             0, 4, 1, 5, 2, 6, 3, 7;
    return edges;
}

const char* method_name(ContouringMethod method) {
    switch (method) {
        case ContouringMethod::MarchingCubes: return "Marching Cubes";
        case ContouringMethod::DualContouring: return "Dual Contouring";
        case ContouringMethod::Ours: return "Ours";
    }
    return "Unknown";
}

} // namespace

Application::Application() {
    options_.method = ContouringMethod::Ours;
    progress_.stage = ContouringStage::Validating;
    progress_.message = "Idle";
}

Application::~Application() {
    cancel_generation();
    if (worker_.joinable()) worker_.join();
}

void Application::initialize(const std::string& initial_path) {
    update_crop_preview();
    if (!initial_path.empty()) load_mesh(initial_path);
}

void Application::handle_dropped_files(const std::vector<std::string>& paths) {
    if (!paths.empty() && !worker_running_) load_mesh(paths.front());
}

void Application::load_mesh(const std::string& path) {
    if (worker_running_) return;
    Eigen::MatrixXd vertices;
    Eigen::MatrixXi faces;
    if (!igl::read_triangle_mesh(path, vertices, faces) || vertices.rows() == 0 || faces.rows() == 0) {
        last_error_ = "Could not read a non-empty triangle mesh from: " + path;
        return;
    }
    if (vertices.cols() != 3 || faces.cols() != 3 || !vertices.allFinite()) {
        last_error_ = "The selected file does not contain a valid 3D triangle mesh.";
        return;
    }

    source_vertices_ = std::move(vertices);
    source_faces_ = std::move(faces);
    source_path_ = path;
    Eigen::MatrixXi boundary_edges;
    igl::boundary_facets(source_faces_, boundary_edges);
    const bool edge_manifold = igl::is_edge_manifold(source_faces_);
    if (!edge_manifold) {
        source_warning_ = "The source is not edge-manifold; signed-distance signs may be unreliable.";
    } else if (boundary_edges.rows() > 0) {
        source_warning_ = "The source has open boundaries; signed-distance signs may be unreliable.";
    } else {
        source_warning_.clear();
    }
    source_pivot_ = 0.5 * (
        source_vertices_.colwise().minCoeff().transpose() +
        source_vertices_.colwise().maxCoeff().transpose()
    );
    normalize_source();
    reset_crop_bounds();
    update_source_preview();
    update_crop_preview();
    last_error_.clear();
    status_message_ = "Mesh loaded. Adjust settings, then click Generate.";
    dirty_ = true;
}

void Application::reset_transform() {
    transform_ = {};
    dirty_ = true;
    update_source_preview();
}

void Application::normalize_source() {
    if (source_vertices_.rows() == 0) return;
    const Eigen::Vector3d minimum = source_vertices_.colwise().minCoeff().transpose();
    const Eigen::Vector3d maximum = source_vertices_.colwise().maxCoeff().transpose();
    source_pivot_ = 0.5 * (minimum + maximum);
    const double maximum_extent = (maximum - minimum).maxCoeff();
    const float scale = maximum_extent > 0.0 ? static_cast<float>(2.0 / maximum_extent) : 1.0f;
    transform_ = {};
    transform_.scale = {scale, scale, scale};
    transform_.translation = {
        static_cast<float>(-source_pivot_.x()),
        static_cast<float>(-source_pivot_.y()),
        static_cast<float>(-source_pivot_.z())
    };
    dirty_ = true;
    update_source_preview();
}

Eigen::MatrixXd Application::transformed_source_vertices() const {
    if (source_vertices_.rows() == 0) return {};
    Eigen::MatrixXd transformed = source_vertices_.rowwise() - source_pivot_.transpose();
    transformed.col(0) *= transform_.scale[0];
    transformed.col(1) *= transform_.scale[1];
    transformed.col(2) *= transform_.scale[2];
    transformed = transformed * rotation_matrix(transform_.rotation_degrees).transpose();
    const Eigen::Vector3d translation(
        transform_.translation[0],
        transform_.translation[1],
        transform_.translation[2]
    );
    transformed.rowwise() += (source_pivot_ + translation).transpose();
    return transformed;
}

void Application::reset_crop_bounds() {
    const Eigen::MatrixXd vertices = transformed_source_vertices();
    if (vertices.rows() == 0) return;
    Eigen::Vector3d minimum = vertices.colwise().minCoeff().transpose();
    Eigen::Vector3d maximum = vertices.colwise().maxCoeff().transpose();
    Eigen::Vector3d extent = maximum - minimum;
    for (int axis = 0; axis < 3; ++axis) {
        const double padding = std::max(extent[axis] * crop_padding_, 1e-3);
        bounds_min_[axis] = static_cast<float>(minimum[axis] - padding);
        bounds_max_[axis] = static_cast<float>(maximum[axis] + padding);
    }
    dirty_ = true;
    update_crop_preview();
}

void Application::update_source_preview() {
    if (source_vertices_.rows() == 0) return;
    const Eigen::MatrixXd vertices = transformed_source_vertices();
    polyscope::removeStructure(kSourceMeshName, false);
    auto* mesh = polyscope::registerSurfaceMesh(kSourceMeshName, vertices, source_faces_);
    mesh->setSurfaceColor({source_color_[0], source_color_[1], source_color_[2]});
    mesh->setSmoothShade(source_smooth_);
    mesh->setEdgeWidth(source_wireframe_ ? 1.0 : 0.0);
    mesh->setEnabled(source_visible_);
}

void Application::update_crop_preview() {
    const Eigen::Vector3d minimum(bounds_min_[0], bounds_min_[1], bounds_min_[2]);
    const Eigen::Vector3d maximum(bounds_max_[0], bounds_max_[1], bounds_max_[2]);
    polyscope::removeStructure(kCropBoxName, false);
    auto* box = polyscope::registerCurveNetwork(kCropBoxName, crop_box_vertices(minimum, maximum), crop_box_edges());
    box->setColor({0.1f, 0.65f, 0.95f});
    box->setRadius(0.0025f, false);
    box->setEnabled(crop_visible_);
}

bool Application::validate_generation_request(std::string& error) const {
    if (source_vertices_.rows() == 0) {
        error = "Load a source mesh first.";
        return false;
    }
    for (int axis = 0; axis < 3; ++axis) {
        if (!(bounds_max_[axis] > bounds_min_[axis])) {
            error = "Each crop maximum must be greater than its minimum.";
            return false;
        }
        if (resolution_[axis] < 8 || resolution_[axis] > 257) {
            error = "Grid resolution must be between 8 and 257.";
            return false;
        }
        if (std::abs(transform_.scale[axis]) < 1e-6f) {
            error = "Mesh scale cannot be zero.";
            return false;
        }
    }
    return true;
}

void Application::start_generation() {
    std::string validation_error;
    if (!validate_generation_request(validation_error)) {
        last_error_ = validation_error;
        return;
    }
    if (worker_running_) return;
    if (worker_.joinable()) worker_.join();

    GenerationRequest request;
    request.source_vertices = transformed_source_vertices();
    request.source_faces = source_faces_;
    request.bounds_min = Eigen::Vector3d(bounds_min_[0], bounds_min_[1], bounds_min_[2]);
    request.bounds_max = Eigen::Vector3d(bounds_max_[0], bounds_max_[1], bounds_max_[2]);
    request.resolution = resolution_;
    request.iso_value = iso_value_;
    request.options = options_;

    cancel_requested_ = false;
    worker_running_ = true;
    last_error_.clear();
    status_message_ = "Generating mesh...";
    set_progress(ContouringStage::SamplingSdf, 0.0, "Preparing SDF samples");
    worker_ = std::thread(&Application::generation_worker, this, std::move(request));
}

void Application::cancel_generation() {
    cancel_requested_ = true;
}

void Application::set_progress(ContouringStage stage, double fraction, const std::string& message) {
    std::lock_guard<std::mutex> lock(worker_mutex_);
    progress_.stage = stage;
    progress_.completed = static_cast<int>(fraction * 1000.0);
    progress_.total = 1000;
    progress_.fraction = std::clamp(fraction, 0.0, 1.0);
    progress_.message = message;
}

void Application::generation_worker(GenerationRequest request) {
    GenerationResult result;
    const auto start = std::chrono::steady_clock::now();
    try {
        const int res_x = request.resolution[0];
        const int res_y = request.resolution[1];
        const int res_z = request.resolution[2];
        const long long sample_count = static_cast<long long>(res_x) * res_y * res_z;
        Eigen::MatrixXd grid(sample_count, 3);

        for (int k = 0; k < res_z; ++k) {
            const double z = request.bounds_min.z() +
                (request.bounds_max.z() - request.bounds_min.z()) * k / (res_z - 1.0);
            for (int j = 0; j < res_y; ++j) {
                const double y = request.bounds_min.y() +
                    (request.bounds_max.y() - request.bounds_min.y()) * j / (res_y - 1.0);
                for (int i = 0; i < res_x; ++i) {
                    const double x = request.bounds_min.x() +
                        (request.bounds_max.x() - request.bounds_min.x()) * i / (res_x - 1.0);
                    const long long index = i + static_cast<long long>(res_x) * (j + static_cast<long long>(res_y) * k);
                    grid.row(index) << x, y, z;
                }
            }
            set_progress(ContouringStage::SamplingSdf, 0.1 * (k + 1.0) / res_z, "Building sampling grid");
            if (cancel_requested_) {
                result.cancelled = true;
                break;
            }
        }

        Eigen::VectorXd samples;
        if (!result.cancelled) {
            Eigen::VectorXi closest_faces;
            Eigen::MatrixXd closest_points;
            Eigen::MatrixXd closest_normals;
            set_progress(ContouringStage::SamplingSdf, 0.15, "Sampling signed distance field");
            igl::signed_distance(
                grid,
                request.source_vertices,
                request.source_faces,
                igl::SIGNED_DISTANCE_TYPE_PSEUDONORMAL,
                samples,
                closest_faces,
                closest_points,
                closest_normals
            );
            if (cancel_requested_) result.cancelled = true;
        }

        if (!result.cancelled) {
            ContouringCallbacks callbacks;
            callbacks.cancel_requested = [this]() { return cancel_requested_.load(); };
            callbacks.progress = [this](const ContouringProgress& progress) {
                set_progress(progress.stage, progress.fraction, progress.message);
            };
            const ContouringStatus status = contouring(
                samples,
                grid,
                res_x,
                res_y,
                res_z,
                request.iso_value,
                result.vertices,
                result.faces,
                request.options,
                callbacks
            );
            result.cancelled = status == ContouringStatus::Cancelled;
        }
    } catch (const std::exception& exception) {
        result.error = exception.what();
    } catch (...) {
        result.error = "Unknown generation failure.";
    }

    result.elapsed_seconds = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start
    ).count();
    {
        std::lock_guard<std::mutex> lock(worker_mutex_);
        pending_result_ = std::move(result);
        result_ready_ = true;
    }
    worker_running_ = false;
}

void Application::poll_generation_result() {
    if (!result_ready_.load()) return;
    GenerationResult result;
    {
        std::lock_guard<std::mutex> lock(worker_mutex_);
        if (!result_ready_) return;
        result = std::move(pending_result_);
        result_ready_ = false;
    }
    if (worker_.joinable()) worker_.join();

    last_elapsed_seconds_ = result.elapsed_seconds;
    if (!result.error.empty()) {
        last_error_ = result.error;
        status_message_ = "Generation failed.";
    } else if (result.cancelled) {
        status_message_ = "Generation cancelled. The previous result is unchanged.";
    } else {
        generated_vertices_ = result.vertices;
        generated_faces_ = result.faces;
        register_generated_mesh(result);
        dirty_ = false;
        std::ostringstream stream;
        stream << "Generated " << result.vertices.rows() << " vertices and "
               << result.faces.rows() << " faces in " << result.elapsed_seconds << " s.";
        status_message_ = stream.str();
    }
}

void Application::register_generated_mesh(const GenerationResult& result) {
    polyscope::removeStructure(kGeneratedMeshName, false);
    auto* mesh = polyscope::registerSurfaceMesh(kGeneratedMeshName, result.vertices, result.faces);
    mesh->setSurfaceColor({result_color_[0], result_color_[1], result_color_[2]});
    mesh->setSmoothShade(result_smooth_);
    mesh->setEdgeWidth(result_wireframe_ ? 1.0 : 0.0);
    mesh->setEnabled(result_visible_);
}

std::string Application::choose_mesh_file() {
#ifdef __APPLE__
    return run_command("osascript -e 'POSIX path of (choose file with prompt \"Open triangle mesh\" of type {\"public.3d-content\", \"public.data\"})' 2>/dev/null");
#else
    return {};
#endif
}

std::string Application::choose_export_file() {
#ifdef __APPLE__
    return run_command("osascript -e 'POSIX path of (choose file name with prompt \"Export generated mesh\" default name \"generated.obj\")' 2>/dev/null");
#else
    return {};
#endif
}

void Application::export_generated_mesh() {
    if (generated_vertices_.rows() == 0) return;
    std::string path = choose_export_file();
    if (path.empty()) return;
    if (std::filesystem::path(path).extension().empty()) path += ".obj";
    if (!igl::writeOBJ(path, generated_vertices_, generated_faces_)) {
        last_error_ = "Could not write OBJ file: " + path;
        return;
    }
    status_message_ = "Exported generated mesh to " + path;
}

void Application::draw_ui() {
    poll_generation_result();
    draw_source_panel();
    draw_sampling_panel();
    draw_algorithm_panel();
    draw_generation_panel();
    draw_view_panel();
}

void Application::draw_source_panel() {
    if (!ImGui::CollapsingHeader("Source Mesh", ImGuiTreeNodeFlags_DefaultOpen)) return;
    if (ImGui::Button("Open Mesh...")) {
        const std::string path = choose_mesh_file();
        if (!path.empty()) load_mesh(path);
    }
    if (!source_path_.empty()) {
        ImGui::TextWrapped("%s", source_path_.c_str());
        ImGui::Text("%lld vertices, %lld triangles",
            static_cast<long long>(source_vertices_.rows()),
            static_cast<long long>(source_faces_.rows()));
        if (!source_warning_.empty()) {
            ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.2f, 1.0f), "%s", source_warning_.c_str());
        }
    }
    if (source_vertices_.rows() == 0) return;

    bool changed = false;
    changed |= ImGui::DragFloat3("Translation", transform_.translation.data(), 0.01f);
    changed |= ImGui::DragFloat3("Rotation (degrees)", transform_.rotation_degrees.data(), 0.5f);
    changed |= ImGui::DragFloat3("Scale", transform_.scale.data(), 0.01f, -100.0f, 100.0f);
    if (changed) {
        dirty_ = true;
        update_source_preview();
    }
    if (ImGui::Button("Reset Transform")) reset_transform();
    ImGui::SameLine();
    if (ImGui::Button("Normalize")) {
        normalize_source();
        reset_crop_bounds();
    }
}

void Application::draw_sampling_panel() {
    if (!ImGui::CollapsingHeader("Sampling", ImGuiTreeNodeFlags_DefaultOpen)) return;
    bool changed = false;
    changed |= ImGui::InputFloat3("Bounds Min", bounds_min_.data(), "%.4f");
    changed |= ImGui::InputFloat3("Bounds Max", bounds_max_.data(), "%.4f");
    if (changed) {
        dirty_ = true;
        update_crop_preview();
    }
    ImGui::DragFloat("Padding", &crop_padding_, 0.01f, 0.0f, 2.0f, "%.2f");
    if (ImGui::Button("Fit Bounds to Mesh")) reset_crop_bounds();

    ImGui::Checkbox("Link XYZ resolution", &link_resolution_);
    if (link_resolution_) {
        int resolution = resolution_[0];
        if (ImGui::SliderInt("Resolution", &resolution, 8, 257)) {
            resolution_.fill(resolution);
            dirty_ = true;
        }
    } else if (ImGui::SliderInt3("Resolution XYZ", resolution_.data(), 8, 257)) {
        dirty_ = true;
    }
    if (ImGui::DragFloat("Iso Value", &iso_value_, 0.001f)) dirty_ = true;
    const long long samples = static_cast<long long>(resolution_[0]) * resolution_[1] * resolution_[2];
    ImGui::Text("%lld samples (%.1f MiB grid + SDF)", samples, samples * 32.0 / (1024.0 * 1024.0));
}

void Application::draw_algorithm_panel() {
    if (!ImGui::CollapsingHeader("Algorithm", ImGuiTreeNodeFlags_DefaultOpen)) return;
    int method = static_cast<int>(options_.method);
    const char* methods[] = {"Marching Cubes", "Dual Contouring", "Ours"};
    if (ImGui::Combo("Method", &method, methods, 3)) {
        options_.method = static_cast<ContouringMethod>(method);
        dirty_ = true;
    }
    ImGui::TextDisabled("Current: %s", method_name(options_.method));

    const bool ours = options_.method == ContouringMethod::Ours;
    ImGui::BeginDisabled(!ours);
    dirty_ |= ImGui::InputInt("Outer Iterations", &options_.outer_iters);
    dirty_ |= ImGui::InputInt("Inner Iterations", &options_.inner_iters);
    dirty_ |= ImGui::Checkbox("Hermite Update", &options_.hermite_update);
    dirty_ |= ImGui::InputDouble("Regularization (mu)", &options_.mu, 0.01, 0.1, "%.6f");
    dirty_ |= ImGui::InputDouble("DC Weight", &options_.dc_weight, 0.01, 0.1, "%.6f");
    dirty_ |= ImGui::InputDouble("Sphere Weight", &options_.sphere_weight, 0.05, 0.5, "%.6f");
    dirty_ |= ImGui::InputDouble("SVD Threshold", &options_.svd_threshold, 0.001, 0.01, "%.6f");
    dirty_ |= ImGui::InputDouble("Hermite Position Blend", &options_.new_hermite_pos_weight, 0.01, 0.1, "%.4f");
    dirty_ |= ImGui::InputDouble("Hermite Normal Blend", &options_.new_hermite_normal_weight, 0.01, 0.1, "%.4f");
    dirty_ |= ImGui::InputDouble("Face Position Blend", &options_.new_face_pos_weight, 0.01, 0.1, "%.4f");
    dirty_ |= ImGui::InputInt("Batch Size", &options_.batch_size);
    ImGui::EndDisabled();
    dirty_ |= ImGui::Checkbox("Verbose Logging", &options_.verbose);

    if (ImGui::Button("Reset Algorithm Defaults")) {
        const ContouringMethod selected_method = options_.method;
        options_ = {};
        options_.method = selected_method;
        dirty_ = true;
    }
}

void Application::draw_generation_panel() {
    if (!ImGui::CollapsingHeader("Generation", ImGuiTreeNodeFlags_DefaultOpen)) return;
    if (worker_running_) {
        ContouringProgress progress;
        {
            std::lock_guard<std::mutex> lock(worker_mutex_);
            progress = progress_;
        }
        ImGui::ProgressBar(static_cast<float>(progress.fraction), ImVec2(-1.0f, 0.0f), progress.message.c_str());
        if (ImGui::Button("Cancel")) cancel_generation();
    } else {
        if (ImGui::Button("Generate", ImVec2(-1.0f, 0.0f))) start_generation();
    }
    if (dirty_ && generated_vertices_.rows() > 0) {
        ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.2f, 1.0f), "Result is stale; settings have changed.");
    }
    ImGui::TextWrapped("%s", status_message_.c_str());
    if (last_elapsed_seconds_ > 0.0) ImGui::Text("Last run: %.3f s", last_elapsed_seconds_);
    if (!last_error_.empty()) {
        ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "%s", last_error_.c_str());
    }
    ImGui::BeginDisabled(generated_vertices_.rows() == 0 || worker_running_);
    if (ImGui::Button("Export OBJ...")) export_generated_mesh();
    ImGui::EndDisabled();
}

void Application::draw_view_panel() {
    if (!ImGui::CollapsingHeader("View")) return;
    bool refresh_source = false;
    bool refresh_result = false;
    refresh_source |= ImGui::Checkbox("Show Source", &source_visible_);
    refresh_source |= ImGui::Checkbox("Source Smooth Shading", &source_smooth_);
    refresh_source |= ImGui::Checkbox("Source Wireframe", &source_wireframe_);
    refresh_source |= ImGui::ColorEdit3("Source Color", source_color_.data());
    refresh_result |= ImGui::Checkbox("Show Result", &result_visible_);
    refresh_result |= ImGui::Checkbox("Result Smooth Shading", &result_smooth_);
    refresh_result |= ImGui::Checkbox("Result Wireframe", &result_wireframe_);
    refresh_result |= ImGui::ColorEdit3("Result Color", result_color_.data());
    if (ImGui::Checkbox("Show Sampling Bounds", &crop_visible_)) update_crop_preview();
    if (refresh_source && source_vertices_.rows() > 0) update_source_preview();
    if (refresh_result && generated_vertices_.rows() > 0) {
        GenerationResult result;
        result.vertices = generated_vertices_;
        result.faces = generated_faces_;
        register_generated_mesh(result);
    }
}
