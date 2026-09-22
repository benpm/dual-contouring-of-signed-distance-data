#include "analytic_sdf.h"

#include <dcsdd/contouring.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr double kOctahedronSize = 0.95;
constexpr double kBoundsMinimum = -1.1;
constexpr double kBoundsMaximum = 1.1;
constexpr double kPi = 3.14159265358979323846;
constexpr int kSupersampling = 2;

struct Arguments {
    std::string output_path = "diamond.png";
    int sample_resolution = 33;
    int width = 640;
    int height = 640;
    dcsdd::Method method = dcsdd::Method::Optimized;
    int outer_iterations = 5;
    int inner_iterations = 10;
};

struct ProjectedVertex {
    double x = 0.0;
    double y = 0.0;
    double inverse_depth = 0.0;
    bool visible = false;
};

struct Image {
    int width = 0;
    int height = 0;
    std::vector<std::uint8_t> pixels;
};

void print_usage(const char* executable) {
    std::cout
        << "Usage: " << executable << " [options]\n"
        << "  --output PATH              PNG output path (default: diamond.png)\n"
        << "  --resolution N             SDF grid resolution (default: 33)\n"
        << "  --width N                  Image width (default: 640)\n"
        << "  --height N                 Image height (default: 640)\n"
        << "  --method optimized|dual|marching-cubes\n"
        << "  --outer-iterations N       Optimized outer iterations (default: 5)\n"
        << "  --inner-iterations N       Optimized inner iterations (default: 10)\n"
        << "  --help\n";
}

int parse_integer(const std::string& value, const char* option) {
    std::size_t consumed = 0;
    const int result = std::stoi(value, &consumed);
    if (consumed != value.size()) throw std::invalid_argument(std::string("Invalid value for ") + option);
    return result;
}

Arguments parse_arguments(int argc, char** argv) {
    Arguments arguments;
    for (int index = 1; index < argc; ++index) {
        const std::string option = argv[index];
        if (option == "--help") {
            print_usage(argv[0]);
            std::exit(0);
        }
        if (index + 1 >= argc) throw std::invalid_argument("Missing value for " + option);
        const std::string value = argv[++index];
        if (option == "--output") arguments.output_path = value;
        else if (option == "--resolution") arguments.sample_resolution = parse_integer(value, "--resolution");
        else if (option == "--width") arguments.width = parse_integer(value, "--width");
        else if (option == "--height") arguments.height = parse_integer(value, "--height");
        else if (option == "--outer-iterations") arguments.outer_iterations = parse_integer(value, "--outer-iterations");
        else if (option == "--inner-iterations") arguments.inner_iterations = parse_integer(value, "--inner-iterations");
        else if (option == "--method") {
            if (value == "optimized") arguments.method = dcsdd::Method::Optimized;
            else if (value == "dual") arguments.method = dcsdd::Method::DualContouring;
            else if (value == "marching-cubes") arguments.method = dcsdd::Method::MarchingCubes;
            else throw std::invalid_argument("Unknown method: " + value);
        } else {
            throw std::invalid_argument("Unknown option: " + option);
        }
    }
    if (arguments.output_path.size() < 4 || arguments.output_path.substr(arguments.output_path.size() - 4) != ".png") {
        throw std::invalid_argument("--output must end in .png");
    }
    if (arguments.sample_resolution < 8 || arguments.sample_resolution > 257) {
        throw std::invalid_argument("--resolution must be between 8 and 257");
    }
    if (arguments.width < 64 || arguments.height < 64 || arguments.width > 4096 || arguments.height > 4096) {
        throw std::invalid_argument("Image dimensions must be between 64 and 4096");
    }
    if (arguments.outer_iterations < 0 || arguments.inner_iterations < 0) {
        throw std::invalid_argument("Iteration counts cannot be negative");
    }
    return arguments;
}

dcsdd::SampleGrid sample_octahedron(int resolution) {
    dcsdd::SampleGrid grid;
    grid.dimensions = Eigen::Vector3i::Constant(resolution);
    grid.iso_value = 0.0;
    const int sample_count = resolution * resolution * resolution;
    grid.samples.resize(sample_count);
    grid.positions.resize(sample_count, 3);
    for (int z = 0; z < resolution; ++z) {
        for (int y = 0; y < resolution; ++y) {
            for (int x = 0; x < resolution; ++x) {
                const int sample_index = x + resolution * (y + resolution * z);
                const Eigen::Vector3d point(
                    kBoundsMinimum + (kBoundsMaximum - kBoundsMinimum) * x / (resolution - 1.0),
                    kBoundsMinimum + (kBoundsMaximum - kBoundsMinimum) * y / (resolution - 1.0),
                    kBoundsMinimum + (kBoundsMaximum - kBoundsMinimum) * z / (resolution - 1.0)
                );
                grid.positions.row(sample_index) = point;
                grid.samples[sample_index] = dcsdd::analytic::octahedron(point, kOctahedronSize);
            }
        }
    }
    return grid;
}

double edge_function(
    const ProjectedVertex& first,
    const ProjectedVertex& second,
    double x,
    double y
) {
    return (x - first.x) * (second.y - first.y) -
        (y - first.y) * (second.x - first.x);
}

std::array<std::uint8_t, 3> shade_triangle(
    const Eigen::Vector3d& first,
    const Eigen::Vector3d& second,
    const Eigen::Vector3d& third
) {
    Eigen::Vector3d normal = (second - first).cross(third - first);
    const double normal_length = normal.norm();
    if (normal_length > 1e-15) normal /= normal_length;
    const Eigen::Vector3d centroid = (first + second + third) / 3.0;
    if (normal.dot(centroid) < 0.0) normal = -normal;
    const Eigen::Vector3d light_direction = Eigen::Vector3d(0.35, 0.8, 0.55).normalized();
    const double diffuse = std::max(normal.dot(light_direction), 0.0);
    const double brightness = 0.28 + 0.72 * diffuse;
    const Eigen::Vector3d base_color(0.70, 0.43, 0.92);
    std::array<std::uint8_t, 3> color{};
    for (int channel = 0; channel < 3; ++channel) {
        color[channel] = static_cast<std::uint8_t>(std::clamp(
            std::lround(255.0 * brightness * base_color[channel]),
            0l,
            255l
        ));
    }
    return color;
}

void rasterize_triangle(
    Image& image,
    std::vector<double>& depth_buffer,
    const std::array<ProjectedVertex, 3>& projected,
    const std::array<std::uint8_t, 3>& color
) {
    if (!projected[0].visible || !projected[1].visible || !projected[2].visible) return;
    const double area = edge_function(projected[0], projected[1], projected[2].x, projected[2].y);
    if (std::abs(area) < 1e-12) return;

    const int minimum_x = std::max(0, static_cast<int>(std::floor(std::min({
        projected[0].x, projected[1].x, projected[2].x
    }))));
    const int maximum_x = std::min(image.width - 1, static_cast<int>(std::ceil(std::max({
        projected[0].x, projected[1].x, projected[2].x
    }))));
    const int minimum_y = std::max(0, static_cast<int>(std::floor(std::min({
        projected[0].y, projected[1].y, projected[2].y
    }))));
    const int maximum_y = std::min(image.height - 1, static_cast<int>(std::ceil(std::max({
        projected[0].y, projected[1].y, projected[2].y
    }))));

    for (int y = minimum_y; y <= maximum_y; ++y) {
        for (int x = minimum_x; x <= maximum_x; ++x) {
            const double sample_x = x + 0.5;
            const double sample_y = y + 0.5;
            const double first_weight = edge_function(projected[1], projected[2], sample_x, sample_y) / area;
            const double second_weight = edge_function(projected[2], projected[0], sample_x, sample_y) / area;
            const double third_weight = 1.0 - first_weight - second_weight;
            if (first_weight < -1e-9 || second_weight < -1e-9 || third_weight < -1e-9) continue;
            const double inverse_depth =
                first_weight * projected[0].inverse_depth +
                second_weight * projected[1].inverse_depth +
                third_weight * projected[2].inverse_depth;
            const std::size_t pixel_index = static_cast<std::size_t>(x + image.width * y);
            if (inverse_depth <= depth_buffer[pixel_index]) continue;
            depth_buffer[pixel_index] = inverse_depth;
            const std::size_t color_index = 3 * pixel_index;
            image.pixels[color_index] = color[0];
            image.pixels[color_index + 1] = color[1];
            image.pixels[color_index + 2] = color[2];
        }
    }
}

Image render_mesh(const dcsdd::Mesh& mesh, int output_width, int output_height) {
    Image high_resolution;
    high_resolution.width = output_width * kSupersampling;
    high_resolution.height = output_height * kSupersampling;
    high_resolution.pixels.resize(
        static_cast<std::size_t>(high_resolution.width * high_resolution.height * 3)
    );
    for (std::size_t index = 0; index < high_resolution.pixels.size(); index += 3) {
        high_resolution.pixels[index] = 13;
        high_resolution.pixels[index + 1] = 17;
        high_resolution.pixels[index + 2] = 23;
    }
    std::vector<double> depth_buffer(
        static_cast<std::size_t>(high_resolution.width * high_resolution.height),
        -std::numeric_limits<double>::infinity()
    );

    const Eigen::Vector3d camera_position(2.35, 1.75, 2.55);
    const Eigen::Vector3d forward = (-camera_position).normalized();
    const Eigen::Vector3d right = forward.cross(Eigen::Vector3d::UnitY()).normalized();
    const Eigen::Vector3d up = right.cross(forward).normalized();
    const double focal_length = 0.5 * high_resolution.height /
        std::tan(38.0 * kPi / 360.0);

    std::vector<ProjectedVertex> projected_vertices(static_cast<std::size_t>(mesh.vertices.rows()));
    for (int vertex = 0; vertex < mesh.vertices.rows(); ++vertex) {
        const Eigen::Vector3d relative = mesh.vertices.row(vertex).transpose() - camera_position;
        const double depth = relative.dot(forward);
        ProjectedVertex& projected = projected_vertices[static_cast<std::size_t>(vertex)];
        if (depth <= 0.05) continue;
        projected.x = 0.5 * high_resolution.width + focal_length * relative.dot(right) / depth;
        projected.y = 0.5 * high_resolution.height - focal_length * relative.dot(up) / depth;
        projected.inverse_depth = 1.0 / depth;
        projected.visible = true;
    }

    const int triangles_per_face = mesh.faces.cols() == 4 ? 2 : 1;
    for (int face = 0; face < mesh.faces.rows(); ++face) {
        const std::array<std::array<int, 3>, 2> triangles{{
            {{mesh.faces(face, 0), mesh.faces(face, 1), mesh.faces(face, 2)}},
            {{mesh.faces(face, 0), mesh.faces(face, 2), mesh.faces(face, mesh.faces.cols() - 1)}}
        }};
        for (int triangle = 0; triangle < triangles_per_face; ++triangle) {
            const std::array<int, 3>& indices = triangles[triangle];
            if (indices[0] < 0 || indices[1] < 0 || indices[2] < 0 ||
                indices[0] >= mesh.vertices.rows() || indices[1] >= mesh.vertices.rows() ||
                indices[2] >= mesh.vertices.rows()) {
                continue;
            }
            const Eigen::Vector3d first = mesh.vertices.row(indices[0]);
            const Eigen::Vector3d second = mesh.vertices.row(indices[1]);
            const Eigen::Vector3d third = mesh.vertices.row(indices[2]);
            rasterize_triangle(
                high_resolution,
                depth_buffer,
                {{
                    projected_vertices[static_cast<std::size_t>(indices[0])],
                    projected_vertices[static_cast<std::size_t>(indices[1])],
                    projected_vertices[static_cast<std::size_t>(indices[2])]
                }},
                shade_triangle(first, second, third)
            );
        }
    }

    Image output;
    output.width = output_width;
    output.height = output_height;
    output.pixels.resize(static_cast<std::size_t>(output_width * output_height * 3));
    for (int y = 0; y < output_height; ++y) {
        for (int x = 0; x < output_width; ++x) {
            for (int channel = 0; channel < 3; ++channel) {
                int sum = 0;
                for (int offset_y = 0; offset_y < kSupersampling; ++offset_y) {
                    for (int offset_x = 0; offset_x < kSupersampling; ++offset_x) {
                        const int source_x = kSupersampling * x + offset_x;
                        const int source_y = kSupersampling * y + offset_y;
                        const std::size_t source_index = static_cast<std::size_t>(
                            channel + 3 * (source_x + high_resolution.width * source_y)
                        );
                        sum += high_resolution.pixels[source_index];
                    }
                }
                output.pixels[static_cast<std::size_t>(channel + 3 * (x + output_width * y))] =
                    static_cast<std::uint8_t>(sum / (kSupersampling * kSupersampling));
            }
        }
    }
    return output;
}

void append_big_endian(std::vector<std::uint8_t>& bytes, std::uint32_t value) {
    bytes.push_back(static_cast<std::uint8_t>((value >> 24) & 0xff));
    bytes.push_back(static_cast<std::uint8_t>((value >> 16) & 0xff));
    bytes.push_back(static_cast<std::uint8_t>((value >> 8) & 0xff));
    bytes.push_back(static_cast<std::uint8_t>(value & 0xff));
}

std::uint32_t crc32(const std::uint8_t* data, std::size_t size) {
    std::uint32_t crc = 0xffffffffu;
    for (std::size_t index = 0; index < size; ++index) {
        crc ^= data[index];
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc >> 1) ^ (0xedb88320u & (0u - (crc & 1u)));
        }
    }
    return crc ^ 0xffffffffu;
}

void append_chunk(
    std::vector<std::uint8_t>& png,
    const std::array<char, 4>& type,
    const std::vector<std::uint8_t>& data
) {
    append_big_endian(png, static_cast<std::uint32_t>(data.size()));
    const std::size_t crc_start = png.size();
    for (char character : type) png.push_back(static_cast<std::uint8_t>(character));
    png.insert(png.end(), data.begin(), data.end());
    append_big_endian(png, crc32(png.data() + crc_start, png.size() - crc_start));
}

std::vector<std::uint8_t> make_zlib_stream(const std::vector<std::uint8_t>& raw_data) {
    std::vector<std::uint8_t> stream;
    stream.reserve(raw_data.size() + raw_data.size() / 65535 * 5 + 16);
    stream.push_back(0x78);
    stream.push_back(0x01);
    std::size_t offset = 0;
    while (offset < raw_data.size()) {
        const std::size_t block_size = std::min<std::size_t>(65535, raw_data.size() - offset);
        const bool final_block = offset + block_size == raw_data.size();
        stream.push_back(final_block ? 0x01 : 0x00);
        stream.push_back(static_cast<std::uint8_t>(block_size & 0xff));
        stream.push_back(static_cast<std::uint8_t>((block_size >> 8) & 0xff));
        const std::uint16_t inverted_size = static_cast<std::uint16_t>(~block_size);
        stream.push_back(static_cast<std::uint8_t>(inverted_size & 0xff));
        stream.push_back(static_cast<std::uint8_t>((inverted_size >> 8) & 0xff));
        stream.insert(
            stream.end(),
            raw_data.begin() + static_cast<std::ptrdiff_t>(offset),
            raw_data.begin() + static_cast<std::ptrdiff_t>(offset + block_size)
        );
        offset += block_size;
    }

    std::uint32_t first_sum = 1;
    std::uint32_t second_sum = 0;
    for (std::uint8_t byte : raw_data) {
        first_sum = (first_sum + byte) % 65521u;
        second_sum = (second_sum + first_sum) % 65521u;
    }
    append_big_endian(stream, (second_sum << 16) | first_sum);
    return stream;
}

void write_png(const Image& image, const std::string& path) {
    std::vector<std::uint8_t> raw_data;
    raw_data.reserve(static_cast<std::size_t>(image.height * (1 + 3 * image.width)));
    for (int y = 0; y < image.height; ++y) {
        raw_data.push_back(0);
        const std::size_t row_start = static_cast<std::size_t>(3 * image.width * y);
        raw_data.insert(
            raw_data.end(),
            image.pixels.begin() + static_cast<std::ptrdiff_t>(row_start),
            image.pixels.begin() + static_cast<std::ptrdiff_t>(row_start + 3 * image.width)
        );
    }

    std::vector<std::uint8_t> png{137, 80, 78, 71, 13, 10, 26, 10};
    std::vector<std::uint8_t> header;
    append_big_endian(header, static_cast<std::uint32_t>(image.width));
    append_big_endian(header, static_cast<std::uint32_t>(image.height));
    header.insert(header.end(), {8, 2, 0, 0, 0});
    append_chunk(png, {{'I', 'H', 'D', 'R'}}, header);
    append_chunk(png, {{'I', 'D', 'A', 'T'}}, make_zlib_stream(raw_data));
    append_chunk(png, {{'I', 'E', 'N', 'D'}}, {});

    std::ofstream output(path, std::ios::binary);
    if (!output) throw std::runtime_error("Could not open output file: " + path);
    output.write(reinterpret_cast<const char*>(png.data()), static_cast<std::streamsize>(png.size()));
    if (!output) throw std::runtime_error("Could not write output file: " + path);
}

void print_mesh_metrics(const dcsdd::Mesh& mesh) {
    double maximum_surface_error = 0.0;
    double maximum_edge_length = 0.0;
    std::map<std::pair<int, int>, int> edge_use_counts;
    for (int vertex = 0; vertex < mesh.vertices.rows(); ++vertex) {
        maximum_surface_error = std::max(
            maximum_surface_error,
            std::abs(dcsdd::analytic::octahedron(mesh.vertices.row(vertex), kOctahedronSize))
        );
    }
    for (int face = 0; face < mesh.faces.rows(); ++face) {
        for (int corner = 0; corner < mesh.faces.cols(); ++corner) {
            int first_index = mesh.faces(face, corner);
            int second_index = mesh.faces(face, (corner + 1) % mesh.faces.cols());
            const Eigen::Vector3d first = mesh.vertices.row(first_index);
            const Eigen::Vector3d second = mesh.vertices.row(second_index);
            maximum_edge_length = std::max(maximum_edge_length, (second - first).norm());
            if (first_index > second_index) std::swap(first_index, second_index);
            ++edge_use_counts[{first_index, second_index}];
        }
    }
    int boundary_edges = 0;
    int non_manifold_edges = 0;
    for (const auto& edge : edge_use_counts) {
        if (edge.second == 1) ++boundary_edges;
        else if (edge.second != 2) ++non_manifold_edges;
    }
    const long long euler_characteristic = static_cast<long long>(mesh.vertices.rows()) -
        static_cast<long long>(edge_use_counts.size()) + static_cast<long long>(mesh.faces.rows());
    std::cout
        << "Mesh: " << mesh.vertices.rows() << " vertices, " << mesh.faces.rows() << " faces\n"
        << "Maximum surface error: " << maximum_surface_error << '\n'
        << "Maximum edge length: " << maximum_edge_length << '\n'
        << "Boundary edges: " << boundary_edges << '\n'
        << "Non-manifold edges: " << non_manifold_edges << '\n'
        << "Euler characteristic: " << euler_characteristic << '\n';
}

} // namespace

int main(int argc, char** argv) {
    try {
        const Arguments arguments = parse_arguments(argc, argv);
        dcsdd::Options options;
        options.method = arguments.method;
        options.outer_iterations = arguments.outer_iterations;
        options.inner_iterations = arguments.inner_iterations;
        dcsdd::Mesh mesh;
        if (dcsdd::generate(sample_octahedron(arguments.sample_resolution), mesh, options) !=
            dcsdd::Status::Completed) {
            throw std::runtime_error("Mesh generation was cancelled");
        }
        print_mesh_metrics(mesh);
        write_png(render_mesh(mesh, arguments.width, arguments.height), arguments.output_path);
        std::cout << "Screenshot: " << arguments.output_path << '\n';
        return 0;
    } catch (const std::exception& exception) {
        std::cerr << "Error: " << exception.what() << '\n';
        return 1;
    }
}
