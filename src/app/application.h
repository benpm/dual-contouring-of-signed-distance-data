#pragma once

#include "contouring.h"

#include <Eigen/Core>

#include <array>
#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

class Application {
public:
    Application();
    ~Application();

    void initialize(const std::string& initial_path = {});
    void draw_ui();
    void load_mesh(const std::string& path);
    void handle_dropped_files(const std::vector<std::string>& paths);

private:
    struct TransformState {
        std::array<float, 3> translation{0.0f, 0.0f, 0.0f};
        std::array<float, 3> rotation_degrees{0.0f, 0.0f, 0.0f};
        std::array<float, 3> scale{1.0f, 1.0f, 1.0f};
    };

    struct GenerationRequest {
        Eigen::MatrixXd source_vertices;
        Eigen::MatrixXi source_faces;
        Eigen::Vector3d bounds_min;
        Eigen::Vector3d bounds_max;
        std::array<int, 3> resolution;
        double iso_value = 0.0;
        ContouringOptions options;
    };

    struct GenerationResult {
        Eigen::MatrixXd vertices;
        Eigen::MatrixXi faces;
        double elapsed_seconds = 0.0;
        bool cancelled = false;
        std::string error;
    };

    void draw_source_panel();
    void draw_sampling_panel();
    void draw_algorithm_panel();
    void draw_generation_panel();
    void draw_view_panel();

    void normalize_source();
    void reset_transform();
    void reset_crop_bounds();
    void update_source_preview();
    void update_crop_preview();
    Eigen::MatrixXd transformed_source_vertices() const;
    bool validate_generation_request(std::string& error) const;

    void start_generation();
    void cancel_generation();
    void generation_worker(GenerationRequest request);
    void poll_generation_result();
    void register_generated_mesh(const GenerationResult& result);
    void export_generated_mesh();

    void set_progress(ContouringStage stage, double fraction, const std::string& message);
    static std::string choose_mesh_file();
    static std::string choose_export_file();

    Eigen::MatrixXd source_vertices_;
    Eigen::MatrixXi source_faces_;
    Eigen::Vector3d source_pivot_ = Eigen::Vector3d::Zero();
    std::string source_path_;
    std::string source_warning_;
    TransformState transform_;

    std::array<float, 3> bounds_min_{-1.1f, -1.1f, -1.1f};
    std::array<float, 3> bounds_max_{1.1f, 1.1f, 1.1f};
    std::array<int, 3> resolution_{65, 65, 65};
    bool link_resolution_ = true;
    float crop_padding_ = 0.1f;
    float iso_value_ = 0.0f;
    ContouringOptions options_;

    bool source_visible_ = true;
    bool result_visible_ = true;
    bool crop_visible_ = true;
    bool source_smooth_ = false;
    bool result_smooth_ = false;
    bool source_wireframe_ = true;
    bool result_wireframe_ = true;
    std::array<float, 3> source_color_{0.5f, 0.5f, 0.52f};
    std::array<float, 3> result_color_{0.87f, 0.2f, 0.58f};

    bool dirty_ = false;
    std::string status_message_ = "Load a mesh to begin.";
    std::string last_error_;
    Eigen::MatrixXd generated_vertices_;
    Eigen::MatrixXi generated_faces_;
    double last_elapsed_seconds_ = 0.0;

    std::thread worker_;
    std::atomic<bool> cancel_requested_{false};
    std::atomic<bool> worker_running_{false};
    std::mutex worker_mutex_;
    ContouringProgress progress_;
    std::atomic<bool> result_ready_{false};
    GenerationResult pending_result_;
};
