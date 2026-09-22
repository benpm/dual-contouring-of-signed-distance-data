# AGENTS.md

## Scope

These instructions apply to the entire repository.

## Project

This repository builds a native C++17 desktop application for generating meshes from sampled signed distance data. The application uses CMake, libigl, Polyscope, and Dear ImGui.

Keep the project C++-only. Do not add Python packages, bindings, scripts, virtual environments, `uv`, or other Python tooling.

## Build

Configure and build with the preset for the host platform:

```bash
cmake --preset macos-release
cmake --build --preset macos-release
```

Use `linux-release` on Linux and `windows-release` on Windows. Build output is stored under `build/<preset>/`.

Initialize missing submodules with:

```bash
git submodule update --init --recursive
```

Do not commit generated build directories or downloaded dependencies.

## Tests

Build and run the native tests with the matching platform preset:

```bash
cmake --preset macos-release
cmake --build --preset macos-release
ctest --preset macos-release
```

Run the narrowest relevant test first. Before finishing a code change, run the native test suite and `git diff --check` when practical.

## Code Layout

- `src/app/`: desktop application, ImGui controls, file handling, and background generation.
- `src/cpp/`: contouring algorithms and supporting geometry code.
- `include/`: public headers for the contouring library.
- `tests/`: native C++ tests.
- `cmake/`: dependency and CMake support modules.
- `external/`: third-party submodules; treat these as read-only unless a task explicitly requires modifying them.

## C++ Style

- Use C++17 and follow the surrounding file's style.
- Prefer small, focused changes over broad refactors.
- Use descriptive names; avoid one-letter names except for established mathematical notation.
- Use RAII and standard library ownership types. Avoid manual memory management.
- Keep public interfaces in `include/` and implementation details in `src/`.
- Do not add comments that merely restate the code.
- Do not add new dependencies unless the task clearly requires them.

## GUI and Concurrency

- Keep Polyscope and ImGui calls on the main thread.
- Run expensive mesh generation outside the UI thread.
- Pass immutable request snapshots to worker threads rather than reading live UI state.
- Synchronize shared worker state and preserve the previous successful mesh when generation fails or is cancelled.
- Keep progress and cancellation callbacks lightweight and thread-safe.
- Mark generated results stale whenever an input affecting generation changes.
- Validate paths, grid dimensions, bounds, and contouring parameters before starting work.

## Contouring Core

- Keep the contouring library usable without requiring an active GUI scene.
- Do not introduce Polyscope rendering side effects into algorithm code.
- Preserve caller outputs when an operation is cancelled.
- Report long-running work through `ContouringCallbacks`.
- Add or update tests for validation, cancellation, progress, and mesh output when changing contouring behavior.

## Documentation

Update `README.md` when build commands, controls, supported formats, dependencies, or user-visible behavior change.

Use plain English and keep documentation concise.
