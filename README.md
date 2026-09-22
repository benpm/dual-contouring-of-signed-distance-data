# Dual Contouring of Signed Distance Data

This repository contains the code for the SIGGRAPH 2026 paper [Dual Contouring of Signed Distance Data](https://gatc.cs.columbia.edu/projects/dual-contouring-of-signed-distance-data.html), by Xiana Carrera, Ningna Wang, Christopher Batty, Oded Stein, and Silvia Sellán.

> [!CAUTION]
> This code is tested on macOS only. If you encounter issues with other platforms, contact [x.carrera@columbia.edu](mailto:x.carrera@columbia.edu).

The method reconstructs explicit polygonal meshes from discretely sampled signed distance data and is designed to recover sharp features. It optimizes one mesh vertex per active grid cell using only sampled SDF values, without requiring arbitrary function queries, gradient access, or trained models.

![Teaser](images/teaser.png)

## Build

### 1. Clone with submodules

```bash
git clone --recursive https://github.com/xianacarrera/dcsdd.git
cd dcsdd
```

If the repository was cloned without submodules:

```bash
git submodule update --init --recursive
```

### 2. Configure and compile

Install the Xcode command-line tools, CMake 3.21 or newer, and Ninja. Then run:

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

The executable is created at `build/dcsdd_app`.

## Desktop Application

Launch the application with an empty scene:

```bash
./build/dcsdd_app
```

Or load a mesh immediately:

```bash
./build/dcsdd_app data/bunny.obj
```

The Polyscope/ImGui interface supports:

- Loading OBJ, OFF, PLY, and STL triangle meshes through a native file dialog or drag and drop.
- Translation, rotation, per-axis scaling, reset, and normalization controls.
- Editable sampling bounds, padding, linked or independent grid resolution, and iso value.
- Marching Cubes, Dual Contouring, and the paper's optimization method.
- All `ContouringOptions` parameters, including iteration counts, energy weights, Hermite updates, SVD threshold, and batch size.
- Background generation with progress, cancellation, validation errors, and preservation of the last successful result.
- Independent source/result visibility, shading, wireframe, and color controls.
- OBJ export with triangle or quad faces as produced by the selected method.

Parameter changes mark the current result as stale. Click **Generate** to run the current configuration.

## Install

To install the executable under a custom prefix:

```bash
cmake --install build --prefix dist
./dist/bin/dcsdd_app
```

## Tests

```bash
cmake -S . -B build-test -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DDCSDD_BUILD_TESTS=ON
cmake --build build-test
ctest --test-dir build-test --output-on-failure
```

## Key Parameters

| Parameter | Default | Description |
|---|---:|---|
| `outer_iters` | 100 | Outer optimization iterations |
| `inner_iters` | 100 | Per-cell local minimization iterations |
| `hermite_update` | `true` | Refine Hermite positions and normals between outer iterations |
| `mu` | 0.1 | Regularization weight |
| `dc_weight` | 0.02 | Dual Contouring energy weight |
| `sphere_weight` | 1.0 | Sphere constraint weight |
| `svd_threshold` | 0.01 | Singular-value cutoff used by the local solve |
| `new_hermite_pos_weight` | 0.2 | Hermite position update blend |
| `new_hermite_normal_weight` | 0.2 | Hermite normal update blend |
| `new_face_pos_weight` | 0.2 | Face intersection update blend |
| `batch_size` | 200000 | SDF samples processed per sphere-assignment batch |
| `verbose` | `false` | Print detailed iteration information |

## Citation

If you use this code in your research, cite:

```bibtex
@inproceedings{Carrera2026DCSDD,
  author = {Carrera, Xiana and Wang, Ningna and Batty, Christopher and Stein, Oded and Sell\'{a}n, Silvia},
  title = {Dual Contouring of Signed Distance Data},
  year = {2026},
  isbn = {9798400725548},
  publisher = {Association for Computing Machinery},
  address = {New York, NY, USA},
  url = {https://doi.org/10.1145/3799902.3811116},
  doi = {10.1145/3799902.3811116},
  booktitle = {Proceedings of the Special Interest Group on Computer Graphics and Interactive Techniques Conference Conference Papers},
  articleno = {38},
  numpages = {12},
  series = {SIGGRAPH Conference Papers '26}
}
```

## Acknowledgements

The Geometry and the City lab at Columbia University is supported by gifts from nTop, Adobe, Dandy, and Braid Technologies, and by a sponsored research project from Dreamsports and the Columbia Engineering Interdisciplinary Research Fund. Christopher Batty acknowledges support from the Natural Sciences and Engineering Research Council of Canada (Grant RGPIN-2021-02524). Oded Stein acknowledges support from the National Science Foundation (award #2335493) and a gift from Adobe.
