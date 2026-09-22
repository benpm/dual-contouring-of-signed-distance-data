cmake_minimum_required(VERSION 3.21)

if(APPLE AND EXISTS "/usr/bin/clang++")
    set(CMAKE_CXX_COMPILER "/usr/bin/clang++" CACHE FILEPATH "C++ compiler" FORCE)
endif()
if(APPLE AND EXISTS "/usr/bin/clang")
    set(CMAKE_C_COMPILER "/usr/bin/clang" CACHE FILEPATH "C compiler" FORCE)
endif()

project(dcsdd VERSION 0.1.0 LANGUAGES C CXX)

find_program(DCSDD_SYSTEM_GIT git
    PATHS /usr/bin /usr/local/bin /opt/homebrew/bin
    NO_DEFAULT_PATH
)
if(DCSDD_SYSTEM_GIT)
    set(GIT_EXECUTABLE "${DCSDD_SYSTEM_GIT}" CACHE FILEPATH "Git executable used by FetchContent" FORCE)
endif()

option(DCSDD_BUILD_TESTS "Build native tests" ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the build type" FORCE)
endif()

list(PREPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(libigl)
igl_include(glfw)

if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/external/polyscope/CMakeLists.txt")
    message(FATAL_ERROR
        "Polyscope is missing. Run: git submodule update --init external/polyscope")
endif()
set(POLYSCOPE_BACKEND_OPENGL_MOCK OFF CACHE BOOL "" FORCE)
add_subdirectory(external/polyscope EXCLUDE_FROM_ALL)

set(DCSDD_CORE_SOURCES
    src/cpp/Cell.cpp
    src/cpp/Sphere.cpp
    src/cpp/contouring.cpp
    src/cpp/hermite_update.cpp
    src/cpp/sdf.cpp
    src/cpp/ui.cpp
    src/cpp/vertex_refinement.cpp
)

add_library(contouring ${DCSDD_CORE_SOURCES})
target_include_directories(contouring PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(contouring PUBLIC igl::glfw polyscope)

find_package(Threads REQUIRED)
add_executable(dcsdd_app
    src/app/application.cpp
    src/app/main.cpp
)
target_include_directories(dcsdd_app PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/app"
)
target_link_libraries(dcsdd_app PRIVATE contouring Threads::Threads)
install(TARGETS dcsdd_app RUNTIME DESTINATION bin)

if(DCSDD_BUILD_TESTS)
    include(CTest)
    if(BUILD_TESTING)
        add_executable(contouring_tests tests/contouring_tests.cpp)
        target_link_libraries(contouring_tests PRIVATE contouring)
        add_test(NAME contouring_tests COMMAND contouring_tests)
    endif()
endif()
