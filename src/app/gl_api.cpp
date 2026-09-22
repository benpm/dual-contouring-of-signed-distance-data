#include "gl_api.h"

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <stdexcept>
#include <string>

namespace {

template <typename Function>
Function load_function(const char* name) {
    const auto function = reinterpret_cast<Function>(glfwGetProcAddress(name));
    if (!function) throw std::runtime_error(std::string("Missing OpenGL function: ") + name);
    return function;
}

#define GL_FUNCTIONS(X) \
    X(Viewport, void, gl::Int, gl::Int, gl::Size, gl::Size) \
    X(ClearColor, void, gl::Float, gl::Float, gl::Float, gl::Float) \
    X(Clear, void, gl::Bitfield) \
    X(ClearBufferuiv, void, gl::Enum, gl::Int, const gl::UInt*) \
    X(Enable, void, gl::Enum) \
    X(Disable, void, gl::Enum) \
    X(DepthFunc, void, gl::Enum) \
    X(CullFace, void, gl::Enum) \
    X(ReadPixels, void, gl::Int, gl::Int, gl::Size, gl::Size, gl::Enum, gl::Enum, void*) \
    X(CreateShader, gl::UInt, gl::Enum) \
    X(ShaderSource, void, gl::UInt, gl::Size, const gl::Char* const*, const gl::Int*) \
    X(CompileShader, void, gl::UInt) \
    X(GetShaderiv, void, gl::UInt, gl::Enum, gl::Int*) \
    X(GetShaderInfoLog, void, gl::UInt, gl::Size, gl::Size*, gl::Char*) \
    X(DeleteShader, void, gl::UInt) \
    X(CreateProgram, gl::UInt) \
    X(AttachShader, void, gl::UInt, gl::UInt) \
    X(BindAttribLocation, void, gl::UInt, gl::UInt, const gl::Char*) \
    X(BindFragDataLocation, void, gl::UInt, gl::UInt, const gl::Char*) \
    X(LinkProgram, void, gl::UInt) \
    X(GetProgramiv, void, gl::UInt, gl::Enum, gl::Int*) \
    X(GetProgramInfoLog, void, gl::UInt, gl::Size, gl::Size*, gl::Char*) \
    X(DeleteProgram, void, gl::UInt) \
    X(UseProgram, void, gl::UInt) \
    X(GetUniformLocation, gl::Int, gl::UInt, const gl::Char*) \
    X(UniformMatrix4fv, void, gl::Int, gl::Size, gl::Boolean, const gl::Float*) \
    X(Uniform3f, void, gl::Int, gl::Float, gl::Float, gl::Float) \
    X(Uniform1ui, void, gl::Int, gl::UInt) \
    X(GenVertexArrays, void, gl::Size, gl::UInt*) \
    X(DeleteVertexArrays, void, gl::Size, const gl::UInt*) \
    X(BindVertexArray, void, gl::UInt) \
    X(GenBuffers, void, gl::Size, gl::UInt*) \
    X(DeleteBuffers, void, gl::Size, const gl::UInt*) \
    X(BindBuffer, void, gl::Enum, gl::UInt) \
    X(BufferData, void, gl::Enum, gl::SizePtr, const void*, gl::Enum) \
    X(EnableVertexAttribArray, void, gl::UInt) \
    X(VertexAttribPointer, void, gl::UInt, gl::Int, gl::Enum, gl::Boolean, gl::Size, const void*) \
    X(DrawArrays, void, gl::Enum, gl::Int, gl::Size) \
    X(GenFramebuffers, void, gl::Size, gl::UInt*) \
    X(DeleteFramebuffers, void, gl::Size, const gl::UInt*) \
    X(BindFramebuffer, void, gl::Enum, gl::UInt) \
    X(CheckFramebufferStatus, gl::Enum, gl::Enum) \
    X(FramebufferTexture2D, void, gl::Enum, gl::Enum, gl::Enum, gl::UInt, gl::Int) \
    X(GenTextures, void, gl::Size, gl::UInt*) \
    X(DeleteTextures, void, gl::Size, const gl::UInt*) \
    X(BindTexture, void, gl::Enum, gl::UInt) \
    X(TexImage2D, void, gl::Enum, gl::Int, gl::Int, gl::Size, gl::Size, gl::Int, gl::Enum, gl::Enum, const void*) \
    X(TexParameteri, void, gl::Enum, gl::Enum, gl::Int) \
    X(GenRenderbuffers, void, gl::Size, gl::UInt*) \
    X(DeleteRenderbuffers, void, gl::Size, const gl::UInt*) \
    X(BindRenderbuffer, void, gl::Enum, gl::UInt) \
    X(RenderbufferStorage, void, gl::Enum, gl::Enum, gl::Size, gl::Size) \
    X(FramebufferRenderbuffer, void, gl::Enum, gl::Enum, gl::Enum, gl::UInt)

#define DECLARE(name, result, ...) using name##Function = result (*)(__VA_ARGS__); name##Function p##name = nullptr;
GL_FUNCTIONS(DECLARE)
#undef DECLARE

} // namespace

namespace gl {

bool load() {
#define LOAD(name, result, ...) p##name = load_function<name##Function>("gl" #name);
    GL_FUNCTIONS(LOAD)
#undef LOAD
    return true;
}

#define WRAP_VOID(wrapper, name, signature, arguments) void wrapper signature { p##name arguments; }
WRAP_VOID(viewport, Viewport, (Int x, Int y, Size width, Size height), (x, y, width, height))
WRAP_VOID(clear_color, ClearColor, (Float r, Float g, Float b, Float a), (r, g, b, a))
WRAP_VOID(clear, Clear, (Bitfield mask), (mask))
WRAP_VOID(clear_buffer_uiv, ClearBufferuiv, (Enum b, Int d, const UInt* v), (b, d, v))
WRAP_VOID(enable, Enable, (Enum value), (value))
WRAP_VOID(disable, Disable, (Enum value), (value))
WRAP_VOID(depth_func, DepthFunc, (Enum value), (value))
WRAP_VOID(cull_face, CullFace, (Enum value), (value))
WRAP_VOID(read_pixels, ReadPixels, (Int x, Int y, Size w, Size h, Enum f, Enum t, void* d), (x, y, w, h, f, t, d))
UInt create_shader(Enum type) { return pCreateShader(type); }
WRAP_VOID(shader_source, ShaderSource, (UInt s, Size c, const Char* const* v, const Int* l), (s, c, v, l))
WRAP_VOID(compile_shader, CompileShader, (UInt value), (value))
WRAP_VOID(get_shader_iv, GetShaderiv, (UInt s, Enum n, Int* v), (s, n, v))
WRAP_VOID(get_shader_info_log, GetShaderInfoLog, (UInt s, Size c, Size* l, Char* v), (s, c, l, v))
WRAP_VOID(delete_shader, DeleteShader, (UInt value), (value))
UInt create_program() { return pCreateProgram(); }
WRAP_VOID(attach_shader, AttachShader, (UInt p, UInt s), (p, s))
WRAP_VOID(bind_attrib_location, BindAttribLocation, (UInt p, UInt i, const Char* n), (p, i, n))
WRAP_VOID(bind_frag_data_location, BindFragDataLocation, (UInt p, UInt i, const Char* n), (p, i, n))
WRAP_VOID(link_program, LinkProgram, (UInt value), (value))
WRAP_VOID(get_program_iv, GetProgramiv, (UInt p, Enum n, Int* v), (p, n, v))
WRAP_VOID(get_program_info_log, GetProgramInfoLog, (UInt p, Size c, Size* l, Char* v), (p, c, l, v))
WRAP_VOID(delete_program, DeleteProgram, (UInt value), (value))
WRAP_VOID(use_program, UseProgram, (UInt value), (value))
Int get_uniform_location(UInt p, const Char* n) { return pGetUniformLocation(p, n); }
WRAP_VOID(uniform_matrix4fv, UniformMatrix4fv, (Int l, Size c, Boolean t, const Float* v), (l, c, t, v))
WRAP_VOID(uniform3f, Uniform3f, (Int l, Float x, Float y, Float z), (l, x, y, z))
WRAP_VOID(uniform1ui, Uniform1ui, (Int l, UInt v), (l, v))
WRAP_VOID(gen_vertex_arrays, GenVertexArrays, (Size c, UInt* v), (c, v))
WRAP_VOID(delete_vertex_arrays, DeleteVertexArrays, (Size c, const UInt* v), (c, v))
WRAP_VOID(bind_vertex_array, BindVertexArray, (UInt value), (value))
WRAP_VOID(gen_buffers, GenBuffers, (Size c, UInt* v), (c, v))
WRAP_VOID(delete_buffers, DeleteBuffers, (Size c, const UInt* v), (c, v))
WRAP_VOID(bind_buffer, BindBuffer, (Enum t, UInt v), (t, v))
WRAP_VOID(buffer_data, BufferData, (Enum t, SizePtr s, const void* d, Enum u), (t, s, d, u))
WRAP_VOID(enable_vertex_attrib_array, EnableVertexAttribArray, (UInt value), (value))
WRAP_VOID(vertex_attrib_pointer, VertexAttribPointer, (UInt i, Int s, Enum t, Boolean n, Size stride, const void* p), (i, s, t, n, stride, p))
WRAP_VOID(draw_arrays, DrawArrays, (Enum m, Int f, Size c), (m, f, c))
WRAP_VOID(gen_framebuffers, GenFramebuffers, (Size c, UInt* v), (c, v))
WRAP_VOID(delete_framebuffers, DeleteFramebuffers, (Size c, const UInt* v), (c, v))
WRAP_VOID(bind_framebuffer, BindFramebuffer, (Enum t, UInt v), (t, v))
Enum check_framebuffer_status(Enum target) { return pCheckFramebufferStatus(target); }
WRAP_VOID(framebuffer_texture_2d, FramebufferTexture2D, (Enum t, Enum a, Enum tt, UInt v, Int l), (t, a, tt, v, l))
WRAP_VOID(gen_textures, GenTextures, (Size c, UInt* v), (c, v))
WRAP_VOID(delete_textures, DeleteTextures, (Size c, const UInt* v), (c, v))
WRAP_VOID(bind_texture, BindTexture, (Enum t, UInt v), (t, v))
WRAP_VOID(tex_image_2d, TexImage2D, (Enum t, Int l, Int i, Size w, Size h, Int b, Enum f, Enum ty, const void* p), (t, l, i, w, h, b, f, ty, p))
WRAP_VOID(tex_parameter_i, TexParameteri, (Enum t, Enum n, Int v), (t, n, v))
WRAP_VOID(gen_renderbuffers, GenRenderbuffers, (Size c, UInt* v), (c, v))
WRAP_VOID(delete_renderbuffers, DeleteRenderbuffers, (Size c, const UInt* v), (c, v))
WRAP_VOID(bind_renderbuffer, BindRenderbuffer, (Enum t, UInt v), (t, v))
WRAP_VOID(renderbuffer_storage, RenderbufferStorage, (Enum t, Enum i, Size w, Size h), (t, i, w, h))
WRAP_VOID(framebuffer_renderbuffer, FramebufferRenderbuffer, (Enum t, Enum a, Enum rt, UInt r), (t, a, rt, r))
#undef WRAP_VOID

} // namespace gl
