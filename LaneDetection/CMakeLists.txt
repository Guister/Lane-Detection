cmake_minimum_required(VERSION 3.0.0)
project(LaneDetection VERSION 0.1.0 LANGUAGES CXX)

# The options need to be the same as Open3D's default
# If Open3D is configured and built with custom options, you'll also need to
# specify the same custom options.
option(STATIC_WINDOWS_RUNTIME "Use static (MT/MTd) Windows runtime" ON)
if(STATIC_WINDOWS_RUNTIME)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
else()
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
endif()

include(src/Sources.cmake)

set(Open3DDirectory "/home/guister/open3d_install") #Replace with your local directory
set(Open3DIncludeDirectory "${Open3DDirectory}/include")

list(APPEND CMAKE_PREFIX_PATH ${Open3DDirectory})

# Find installed Open3D, which exports Open3D::Open3D
find_package(Open3D REQUIRED)

add_executable(LaneDetection)
target_sources(LaneDetection PRIVATE ${Sources})
target_include_directories(LaneDetection PUBLIC ${Open3DIncludeDirectory})
target_link_libraries(LaneDetection PRIVATE Open3D::Open3D)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
