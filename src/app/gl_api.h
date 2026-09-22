#pragma once

#include <cstddef>

struct GLFWwindow;

namespace gl {

using Enum = unsigned int;
using Boolean = unsigned char;
using Bitfield = unsigned int;
using Int = int;
using Size = int;
using UInt = unsigned int;
using Float = float;
using Char = char;
using SizePtr = std::ptrdiff_t;

constexpr Boolean False = 0;
constexpr Bitfield ColorBufferBit = 0x00004000;
constexpr Bitfield DepthBufferBit = 0x00000100;
constexpr Enum DepthTest = 0x0B71;
constexpr Enum Less = 0x0201;
constexpr Enum ArrayBuffer = 0x8892;
constexpr Enum StaticDraw = 0x88E4;
constexpr Enum FloatType = 0x1406;
constexpr Enum Triangles = 0x0004;
constexpr Enum VertexShader = 0x8B31;
constexpr Enum FragmentShader = 0x8B30;
constexpr Enum CompileStatus = 0x8B81;
constexpr Enum LinkStatus = 0x8B82;
constexpr Enum InfoLogLength = 0x8B84;
constexpr Enum Framebuffer = 0x8D40;
constexpr Enum FramebufferComplete = 0x8CD5;
constexpr Enum ColorAttachment0 = 0x8CE0;
constexpr Enum DepthAttachment = 0x8D00;
constexpr Enum Texture2D = 0x0DE1;
constexpr Enum R32ui = 0x8236;
constexpr Enum RedInteger = 0x8D94;
constexpr Enum UnsignedInt = 0x1405;
constexpr Enum TextureMinFilter = 0x2801;
constexpr Enum TextureMagFilter = 0x2800;
constexpr Enum Nearest = 0x2600;
constexpr Enum Renderbuffer = 0x8D41;
constexpr Enum DepthComponent24 = 0x81A6;
constexpr Enum Back = 0x0405;
constexpr Enum CullFace = 0x0B44;
constexpr Enum Color = 0x1800;

bool load();

void viewport(Int x, Int y, Size width, Size height);
void clear_color(Float red, Float green, Float blue, Float alpha);
void clear(Bitfield mask);
void clear_buffer_uiv(Enum buffer, Int draw_buffer, const UInt* value);
void enable(Enum capability);
void disable(Enum capability);
void depth_func(Enum function);
void cull_face(Enum mode);
void read_pixels(Int x, Int y, Size width, Size height, Enum format, Enum type, void* data);

UInt create_shader(Enum type);
void shader_source(UInt shader, Size count, const Char* const* strings, const Int* lengths);
void compile_shader(UInt shader);
void get_shader_iv(UInt shader, Enum name, Int* value);
void get_shader_info_log(UInt shader, Size capacity, Size* length, Char* log);
void delete_shader(UInt shader);
UInt create_program();
void attach_shader(UInt program, UInt shader);
void bind_attrib_location(UInt program, UInt index, const Char* name);
void bind_frag_data_location(UInt program, UInt color_number, const Char* name);
void link_program(UInt program);
void get_program_iv(UInt program, Enum name, Int* value);
void get_program_info_log(UInt program, Size capacity, Size* length, Char* log);
void delete_program(UInt program);
void use_program(UInt program);
Int get_uniform_location(UInt program, const Char* name);
void uniform_matrix4fv(Int location, Size count, Boolean transpose, const Float* value);
void uniform3f(Int location, Float x, Float y, Float z);
void uniform1ui(Int location, UInt value);

void gen_vertex_arrays(Size count, UInt* arrays);
void delete_vertex_arrays(Size count, const UInt* arrays);
void bind_vertex_array(UInt array);
void gen_buffers(Size count, UInt* buffers);
void delete_buffers(Size count, const UInt* buffers);
void bind_buffer(Enum target, UInt buffer);
void buffer_data(Enum target, SizePtr size, const void* data, Enum usage);
void enable_vertex_attrib_array(UInt index);
void vertex_attrib_pointer(UInt index, Int size, Enum type, Boolean normalized, Size stride, const void* pointer);
void draw_arrays(Enum mode, Int first, Size count);

void gen_framebuffers(Size count, UInt* framebuffers);
void delete_framebuffers(Size count, const UInt* framebuffers);
void bind_framebuffer(Enum target, UInt framebuffer);
Enum check_framebuffer_status(Enum target);
void framebuffer_texture_2d(Enum target, Enum attachment, Enum texture_target, UInt texture, Int level);
void gen_textures(Size count, UInt* textures);
void delete_textures(Size count, const UInt* textures);
void bind_texture(Enum target, UInt texture);
void tex_image_2d(Enum target, Int level, Int internal_format, Size width, Size height, Int border, Enum format, Enum type, const void* pixels);
void tex_parameter_i(Enum target, Enum name, Int value);
void gen_renderbuffers(Size count, UInt* renderbuffers);
void delete_renderbuffers(Size count, const UInt* renderbuffers);
void bind_renderbuffer(Enum target, UInt renderbuffer);
void renderbuffer_storage(Enum target, Enum internal_format, Size width, Size height);
void framebuffer_renderbuffer(Enum target, Enum attachment, Enum renderbuffer_target, UInt renderbuffer);

} // namespace gl
