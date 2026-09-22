#pragma once

#include "gl_api.h"

#include <dcsdd/contouring.h>

#include <Eigen/Core>

#include <unordered_map>
#include <vector>

struct RenderItem {
    unsigned int id = 0;
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    Eigen::Vector3f color = Eigen::Vector3f::Ones();
    bool visible = true;
};

class Renderer {
public:
    Renderer();
    ~Renderer();

    Renderer(const Renderer&) = delete;
    Renderer& operator=(const Renderer&) = delete;

    void upload(unsigned int id, const dcsdd::Mesh& mesh);
    void remove(unsigned int id);
    void draw(
        const std::vector<RenderItem>& items,
        const Eigen::Matrix4f& view_projection,
        int width,
        int height
    );
    unsigned int pick(
        const std::vector<RenderItem>& items,
        const Eigen::Matrix4f& view_projection,
        int width,
        int height,
        int x,
        int y
    );

private:
    struct GpuMesh {
        gl::UInt vertex_array = 0;
        gl::UInt vertex_buffer = 0;
        gl::Size vertex_count = 0;
    };

    gl::UInt compile_program(const char* vertex_source, const char* fragment_source);
    void draw_items(
        const std::vector<RenderItem>& items,
        const Eigen::Matrix4f& view_projection,
        bool picking
    );
    void resize_picking_buffer(int width, int height);

    std::unordered_map<unsigned int, GpuMesh> meshes_;
    gl::UInt shaded_program_ = 0;
    gl::UInt picking_program_ = 0;
    gl::UInt picking_framebuffer_ = 0;
    gl::UInt picking_texture_ = 0;
    gl::UInt picking_depth_ = 0;
    int picking_width_ = 0;
    int picking_height_ = 0;
};
