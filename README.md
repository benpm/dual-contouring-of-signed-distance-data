# Dual Contouring of Signed Distance Data

This repository contains a compact C++17 implementation of the method from the SIGGRAPH 2026 paper *Dual Contouring of Signed Distance Data* by Xiana Carrera, Ningna Wang, Christopher Batty, Oded Stein, and Silvia Sellán.

The project contains:

- A reusable contouring library for sampled signed distance grids.
- Marching Cubes, Dual Contouring, and the paper's optimized method.
- A small desktop gallery that generates six analytic examples at startup.
- A custom Eigen-based triangle BVH and OpenGL mesh renderer.

Research datasets and result archives are intentionally not included.

![Teaser](images/teaser.png)

## Dependencies

CMake downloads fixed versions of Eigen, GLFW, and Dear ImGui. The application uses the platform OpenGL library. There are no Git submodules, libigl, Polyscope, or GLM dependencies.

## Build

Install CMake 3.21 or newer. macOS and Linux builds also require Ninja.

```bash
cmake --preset macos-release
cmake --build --preset macos-release
```

Use `linux-release` or `windows-release` on the matching platform.

To build only the library and tests:

```bash
cmake -S . -B build/core -G Ninja -DDCSDD_BUILD_APP=OFF -DBUILD_TESTING=ON
cmake --build build/core
ctest --test-dir build/core --output-on-failure
```

## Gallery

Run:

```bash
./build/macos-release/dcsdd_gallery
```

The gallery generates a sphere, box, torus, cylinder, octahedron, and cut sphere on a background thread. Completed meshes appear in a 3×2 grid.

- Left-click a mesh to inspect it.
- Right-drag to orbit.
- Scroll to zoom.
- Click **Back to Gallery** to return.

The application has no model loading, export, or editing controls.

## Library

Include `dcsdd/contouring.h` and link `dcsdd::core`.

```cpp
dcsdd::SampleGrid grid;
grid.samples = samples;
grid.positions = positions;
grid.dimensions = {resolution_x, resolution_y, resolution_z};

dcsdd::Options options;
options.method = dcsdd::Method::Optimized;

dcsdd::Mesh mesh;
const dcsdd::Status status = dcsdd::generate(grid, mesh, options);
```

Generation is transactional: cancellation leaves the caller's existing mesh unchanged.

## Tests

```bash
ctest --preset macos-release
```

## Citation

```bibtex
@inproceedings{Carrera2026DCSDD,
  author = {Carrera, Xiana and Wang, Ningna and Batty, Christopher and Stein, Oded and Sell\'{a}n, Silvia},
  title = {Dual Contouring of Signed Distance Data},
  year = {2026},
  publisher = {Association for Computing Machinery},
  booktitle = {SIGGRAPH Conference Papers},
  doi = {10.1145/3799902.3811116}
}
```
