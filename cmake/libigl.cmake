if(TARGET igl::core)
    return()
endif()

include(FetchContent)

if(NOT TARGET Eigen3::Eigen)
    if(POLICY CMP0169)
        cmake_policy(PUSH)
        cmake_policy(SET CMP0169 OLD)
    endif()
    FetchContent_Declare(
        eigen
        URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
        URL_HASH SHA256=8586084f71f9bde545ee7fa6d00288b264a2b7ac3607b974e54d13e7162c1c72
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )
    FetchContent_GetProperties(eigen)
    if(NOT eigen_POPULATED)
        FetchContent_Populate(eigen)
    endif()
    add_library(Eigen3_Eigen INTERFACE)
    add_library(Eigen3::Eigen ALIAS Eigen3_Eigen)
    target_include_directories(Eigen3_Eigen SYSTEM INTERFACE "${eigen_SOURCE_DIR}")
    if(POLICY CMP0169)
        cmake_policy(POP)
    endif()
endif()

FetchContent_Declare(
    libigl
    URL https://github.com/libigl/libigl/archive/refs/tags/v2.6.0.tar.gz
    URL_HASH SHA256=fe3bf58571cccbef774947261284ccf6b7fdf04fcab5f7181e31931e42a0b14f
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_MakeAvailable(libigl)
