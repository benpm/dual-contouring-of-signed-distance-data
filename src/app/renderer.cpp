#include "renderer.h"

#include <Eigen/Geometry>

#include <array>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr const char* kVertexShader = R"(
#version 150 core
in vec3 position;
in vec3 normal;
uniform mat4 model;
uniform mat4 view_projection;
out vec3 world_normal;
void main() {
    world_normal = normalize(mat3(model) * normal);
    gl_Position = view_projection * model * vec4(position, 1.0);
}
)";

constexpr const char* kShadedFragmentShader = R"(
#version 150 core
in vec3 world_normal;
uniform vec3 color;
out vec4 fragment_color;
void main() {
    vec3 light_direction = normalize(vec3(0.4, 0.8, 0.6));
    float diffuse = max(dot(normalize(world_normal), light_direction), 0.0);
    fragment_color = vec4(color * (0.25 + 0.75 * diffuse), 1.0);
}
)";

constexpr const char* kPickingFragmentShader = R"(
#version 150 core
uniform uint object_id;
out uint fragment_id;
void main() {
    fragment_id = object_id;
}
)";

std::vector<float> make_render_vertices(const dcsdd::Mesh& mesh) {
    std::vector<float> data;
    if (mesh.vertices.cols() != 3 || (mesh.faces.cols() != 3 && mesh.faces.cols() != 4)) return data;
    const int triangles_per_face = mesh.faces.cols() == 4 ? 2 : 1;
    data.reserve(static_cast<std::size_t>(mesh.faces.rows() * triangles_per_face * 18));
    for (int face = 0; face < mesh.faces.rows(); ++face) {
        const std::array<std::array<int, 3>, 2> triangles{{
            {{mesh.faces(face, 0), mesh.faces(face, 1), mesh.faces(face, 2)}},
            {{mesh.faces(face, 0), mesh.faces(face, 2), mesh.faces(face, mesh.faces.cols() - 1)}}
        }};
        for (int triangle = 0; triangle < triangles_per_face; ++triangle) {
            const auto& indices = triangles[triangle];
            if (indices[0] < 0 || indices[1] < 0 || indices[2] < 0 ||
                indices[0] >= mesh.vertices.rows() || indices[1] >= mesh.vertices.rows() ||
                indices[2] >= mesh.vertices.rows()) {
                continue;
            }
            const Eigen::Vector3f a = mesh.vertices.row(indices[0]).cast<float>();
            const Eigen::Vector3f b = mesh.vertices.row(indices[1]).cast<float>();
            const Eigen::Vector3f c = mesh.vertices.row(indices[2]).cast<float>();
            Eigen::Vector3f normal = (b - a).cross(c - a);
            const float length = normal.norm();
            if (length <= 1e-12f) continue;
            normal /= length;
            for (const Eigen::Vector3f& position : {a, b, c}) {
                data.insert(data.end(), {position.x(), position.y(), position.z(), normal.x(), normal.y(), normal.z()});
            }
        }
    }
    return data;
}

gl::UInt compile_shader(gl::Enum type, const char* source) {
    const gl::UInt shader = gl::create_shader(type);
    gl::shader_source(shader, 1, &source, nullptr);
    gl::compile_shader(shader);
    gl::Int compiled = 0;
    gl::get_shader_iv(shader, gl::CompileStatus, &compiled);
    if (compiled) return shader;
    gl::Int length = 0;
    gl::get_shader_iv(shader, gl::InfoLogLength, &length);
    std::string message(static_cast<std::size_t>(std::max(length, 1)), '\0');
    gl::get_shader_info_log(shader, length, nullptr, message.data());
    gl::delete_shader(shader);
    throw std::runtime_error("OpenGL shader compilation failed: " + message);
}

} // namespace

Renderer::Renderer() {
    shaded_program_ = compile_program(kVertexShader, kShadedFragmentShader);
    picking_program_ = compile_program(kVertexShader, kPickingFragmentShader);
    gl::gen_framebuffers(1, &picking_framebuffer_);
    gl::gen_textures(1, &picking_texture_);
    gl::gen_renderbuffers(1, &picking_depth_);
}

Renderer::~Renderer() {
    for (const auto& entry : meshes_) {
        gl::delete_buffers(1, &entry.second.vertex_buffer);
        gl::delete_vertex_arrays(1, &entry.second.vertex_array);
    }
    if (picking_depth_) gl::delete_renderbuffers(1, &picking_depth_);
    if (picking_texture_) gl::delete_textures(1, &picking_texture_);
    if (picking_framebuffer_) gl::delete_framebuffers(1, &picking_framebuffer_);
    if (picking_program_) gl::delete_program(picking_program_);
    if (shaded_program_) gl::delete_program(shaded_program_);
}

gl::UInt Renderer::compile_program(const char* vertex_source, const char* fragment_source) {
    const gl::UInt vertex_shader = compile_shader(gl::VertexShader, vertex_source);
    const gl::UInt fragment_shader = compile_shader(gl::FragmentShader, fragment_source);
    const gl::UInt program = gl::create_program();
    gl::attach_shader(program, vertex_shader);
    gl::attach_shader(program, fragment_shader);
    gl::bind_attrib_location(program, 0, "position");
    gl::bind_attrib_location(program, 1, "normal");
    gl::bind_frag_data_location(program, 0, "fragment_color");
    gl::bind_frag_data_location(program, 0, "fragment_id");
    gl::link_program(program);
    gl::delete_shader(vertex_shader);
    gl::delete_shader(fragment_shader);
    gl::Int linked = 0;
    gl::get_program_iv(program, gl::LinkStatus, &linked);
    if (linked) return program;
    gl::Int length = 0;
    gl::get_program_iv(program, gl::InfoLogLength, &length);
    std::string message(static_cast<std::size_t>(std::max(length, 1)), '\0');
    gl::get_program_info_log(program, length, nullptr, message.data());
    gl::delete_program(program);
    throw std::runtime_error("OpenGL program link failed: " + message);
}

void Renderer::upload(unsigned int id, const dcsdd::Mesh& mesh) {
    remove(id);
    const std::vector<float> vertices = make_render_vertices(mesh);
    if (vertices.empty()) return;
    GpuMesh gpu_mesh;
    gpu_mesh.vertex_count = static_cast<gl::Size>(vertices.size() / 6);
    gl::gen_vertex_arrays(1, &gpu_mesh.vertex_array);
    gl::gen_buffers(1, &gpu_mesh.vertex_buffer);
    gl::bind_vertex_array(gpu_mesh.vertex_array);
    gl::bind_buffer(gl::ArrayBuffer, gpu_mesh.vertex_buffer);
    gl::buffer_data(
        gl::ArrayBuffer,
        static_cast<gl::SizePtr>(vertices.size() * sizeof(float)),
        vertices.data(),
        gl::StaticDraw
    );
    gl::enable_vertex_attrib_array(0);
    gl::vertex_attrib_pointer(0, 3, gl::FloatType, gl::False, 6 * sizeof(float), nullptr);
    gl::enable_vertex_attrib_array(1);
    gl::vertex_attrib_pointer(1, 3, gl::FloatType, gl::False, 6 * sizeof(float), reinterpret_cast<void*>(3 * sizeof(float)));
    gl::bind_vertex_array(0);
    meshes_.emplace(id, gpu_mesh);
}

void Renderer::remove(unsigned int id) {
    const auto found = meshes_.find(id);
    if (found == meshes_.end()) return;
    gl::delete_buffers(1, &found->second.vertex_buffer);
    gl::delete_vertex_arrays(1, &found->second.vertex_array);
    meshes_.erase(found);
}

void Renderer::draw(
    const std::vector<RenderItem>& items,
    const Eigen::Matrix4f& view_projection,
    int width,
    int height
) {
    gl::bind_framebuffer(gl::Framebuffer, 0);
    gl::viewport(0, 0, width, height);
    gl::clear_color(0.055f, 0.065f, 0.085f, 1.0f);
    gl::clear(gl::ColorBufferBit | gl::DepthBufferBit);
    gl::enable(gl::DepthTest);
    gl::depth_func(gl::Less);
    gl::disable(gl::CullFace);
    draw_items(items, view_projection, false);
}

void Renderer::draw_items(
    const std::vector<RenderItem>& items,
    const Eigen::Matrix4f& view_projection,
    bool picking
) {
    const gl::UInt program = picking ? picking_program_ : shaded_program_;
    gl::use_program(program);
    const gl::Int model_location = gl::get_uniform_location(program, "model");
    const gl::Int view_projection_location = gl::get_uniform_location(program, "view_projection");
    const gl::Int color_location = picking ? -1 : gl::get_uniform_location(program, "color");
    const gl::Int id_location = picking ? gl::get_uniform_location(program, "object_id") : -1;
    gl::uniform_matrix4fv(view_projection_location, 1, gl::False, view_projection.data());
    for (const RenderItem& item : items) {
        if (!item.visible) continue;
        const auto found = meshes_.find(item.id);
        if (found == meshes_.end()) continue;
        gl::uniform_matrix4fv(model_location, 1, gl::False, item.transform.data());
        if (picking) gl::uniform1ui(id_location, item.id);
        else gl::uniform3f(color_location, item.color.x(), item.color.y(), item.color.z());
        gl::bind_vertex_array(found->second.vertex_array);
        gl::draw_arrays(gl::Triangles, 0, found->second.vertex_count);
    }
    gl::bind_vertex_array(0);
    gl::use_program(0);
}

void Renderer::resize_picking_buffer(int width, int height) {
    if (width == picking_width_ && height == picking_height_) return;
    picking_width_ = width;
    picking_height_ = height;
    gl::bind_texture(gl::Texture2D, picking_texture_);
    gl::tex_image_2d(gl::Texture2D, 0, gl::R32ui, width, height, 0, gl::RedInteger, gl::UnsignedInt, nullptr);
    gl::tex_parameter_i(gl::Texture2D, gl::TextureMinFilter, gl::Nearest);
    gl::tex_parameter_i(gl::Texture2D, gl::TextureMagFilter, gl::Nearest);
    gl::bind_renderbuffer(gl::Renderbuffer, picking_depth_);
    gl::renderbuffer_storage(gl::Renderbuffer, gl::DepthComponent24, width, height);
    gl::bind_framebuffer(gl::Framebuffer, picking_framebuffer_);
    gl::framebuffer_texture_2d(gl::Framebuffer, gl::ColorAttachment0, gl::Texture2D, picking_texture_, 0);
    gl::framebuffer_renderbuffer(gl::Framebuffer, gl::DepthAttachment, gl::Renderbuffer, picking_depth_);
    if (gl::check_framebuffer_status(gl::Framebuffer) != gl::FramebufferComplete) {
        throw std::runtime_error("Could not create picking framebuffer");
    }
    gl::bind_framebuffer(gl::Framebuffer, 0);
}

unsigned int Renderer::pick(
    const std::vector<RenderItem>& items,
    const Eigen::Matrix4f& view_projection,
    int width,
    int height,
    int x,
    int y
) {
    if (width <= 0 || height <= 0 || x < 0 || x >= width || y < 0 || y >= height) return 0;
    resize_picking_buffer(width, height);
    gl::bind_framebuffer(gl::Framebuffer, picking_framebuffer_);
    gl::viewport(0, 0, width, height);
    const gl::UInt clear_value = 0;
    gl::clear_buffer_uiv(gl::Color, 0, &clear_value);
    gl::clear(gl::DepthBufferBit);
    gl::enable(gl::DepthTest);
    gl::disable(gl::CullFace);
    draw_items(items, view_projection, true);
    gl::UInt result = 0;
    gl::read_pixels(x, height - 1 - y, 1, 1, gl::RedInteger, gl::UnsignedInt, &result);
    gl::bind_framebuffer(gl::Framebuffer, 0);
    return result;
}
