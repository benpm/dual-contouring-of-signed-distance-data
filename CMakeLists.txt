cmake_minimum_required(VERSION 3.21)

project(dcsdd VERSION 0.2.0 LANGUAGES CXX)

option(DCSDD_BUILD_APP "Build the desktop gallery" ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)

FetchContent_Declare(
    eigen
    URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
    URL_HASH SHA256=8586084f71f9bde545ee7fa6d00288b264a2b7ac3607b974e54d13e7162c1c72
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
if(POLICY CMP0169)
    cmake_policy(PUSH)
    cmake_policy(SET CMP0169 OLD)
endif()
FetchContent_GetProperties(eigen)
if(NOT eigen_POPULATED)
    FetchContent_Populate(eigen)
endif()
if(POLICY CMP0169)
    cmake_policy(POP)
endif()
add_library(eigen_headers INTERFACE)
add_library(Eigen3::Eigen ALIAS eigen_headers)
target_include_directories(eigen_headers SYSTEM INTERFACE "${eigen_SOURCE_DIR}")

add_library(dcsdd_core
    src/cpp/Cell.cpp
    src/cpp/Sphere.cpp
    src/cpp/analytic_sdf.cpp
    src/cpp/api.cpp
    src/cpp/contouring.cpp
    src/cpp/geometry.cpp
    src/cpp/hermite_update.cpp
    src/cpp/vertex_refinement.cpp
)
add_library(dcsdd::core ALIAS dcsdd_core)
target_include_directories(dcsdd_core
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include"
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src/cpp"
)
target_link_libraries(dcsdd_core PUBLIC Eigen3::Eigen)

add_executable(dcsdd_screenshot src/tools/screenshot.cpp)
target_include_directories(dcsdd_screenshot PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src/cpp")
target_link_libraries(dcsdd_screenshot PRIVATE dcsdd::core)
install(TARGETS dcsdd_screenshot RUNTIME DESTINATION bin)

if(DCSDD_BUILD_APP)
    enable_language(C)
    FetchContent_Declare(
        glfw
        URL https://github.com/glfw/glfw/archive/refs/tags/3.3.9.tar.gz
        URL_HASH SHA256=a7e7faef424fcb5f83d8faecf9d697a338da7f7a906fc1afbc0e1879ef31bd53
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )
    set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(glfw)

    FetchContent_Declare(
        imgui
        URL https://github.com/ocornut/imgui/archive/922a11f0847fe1a7907345c31a22e0e77a43829c.tar.gz
        URL_HASH SHA256=0afe1d3cb77b5dd5252796c79382457e45ede9388939e6ea093d9988d812e97e
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )
    FetchContent_MakeAvailable(imgui)

    find_package(OpenGL REQUIRED)
    add_library(imgui STATIC
        "${imgui_SOURCE_DIR}/imgui.cpp"
        "${imgui_SOURCE_DIR}/imgui_draw.cpp"
        "${imgui_SOURCE_DIR}/imgui_tables.cpp"
        "${imgui_SOURCE_DIR}/imgui_widgets.cpp"
        "${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp"
        "${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp"
    )
    target_include_directories(imgui PUBLIC "${imgui_SOURCE_DIR}")
    target_link_libraries(imgui PUBLIC glfw OpenGL::GL)
    if(APPLE)
        target_compile_definitions(imgui PRIVATE GL_SILENCE_DEPRECATION)
    endif()

    find_package(Threads REQUIRED)
    add_executable(dcsdd_gallery
        src/app/application.cpp
        src/app/gl_api.cpp
        src/app/main.cpp
        src/app/renderer.cpp
    )
    target_include_directories(dcsdd_gallery PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src/app"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/cpp"
    )
    target_link_libraries(dcsdd_gallery PRIVATE dcsdd::core imgui glfw OpenGL::GL Threads::Threads)
    if(APPLE)
        target_compile_definitions(dcsdd_gallery PRIVATE GL_SILENCE_DEPRECATION)
    endif()
    install(TARGETS dcsdd_gallery RUNTIME DESTINATION bin)
endif()

include(CTest)
if(BUILD_TESTING)
    add_executable(contouring_tests tests/contouring_tests.cpp)
    target_include_directories(contouring_tests PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src/cpp")
    target_link_libraries(contouring_tests PRIVATE dcsdd::core)
    add_test(NAME contouring_tests COMMAND contouring_tests)
endif()
