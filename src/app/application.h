#pragma once

#include "renderer.h"

#include <dcsdd/contouring.h>

#include <Eigen/Core>

#include <array>
#include <atomic>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

class Application {
public:
    explicit Application(Renderer& renderer);
    ~Application();

    Application(const Application&) = delete;
    Application& operator=(const Application&) = delete;

    void update();
    void draw_ui();
    void orbit(float horizontal, float vertical);
    void zoom(float amount);
    void select(unsigned int id);
    Eigen::Matrix4f view_projection(float aspect_ratio) const;
    const std::vector<RenderItem>& render_items() const;

private:
    using Sdf = std::function<double(const Eigen::Vector3d&)>;

    struct Example {
        unsigned int id = 0;
        std::string name;
        Sdf sdf;
        Eigen::Vector3f color;
        Eigen::Vector3f gallery_position;
        bool ready = false;
        std::string error;
    };

    struct WorkerResult {
        std::size_t example_index = 0;
        dcsdd::Mesh mesh;
        std::string error;
    };

    void run_worker();
    dcsdd::SampleGrid sample(const Sdf& sdf) const;
    void rebuild_render_items();

    Renderer& renderer_;
    std::vector<Example> examples_;
    std::vector<RenderItem> render_items_;
    unsigned int selected_id_ = 0;

    std::thread worker_;
    std::atomic<bool> cancel_requested_{false};
    std::atomic<bool> worker_finished_{false};
    mutable std::mutex worker_mutex_;
    std::deque<WorkerResult> pending_results_;
    std::string progress_message_ = "Preparing examples";
    double progress_fraction_ = 0.0;

    float camera_yaw_ = 0.0f;
    float camera_pitch_ = 0.25f;
    float camera_distance_ = 9.0f;
};
