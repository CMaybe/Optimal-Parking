# Set the minimum version of CMake required
cmake_minimum_required(VERSION 3.27.4)

# Project name and language
project(to_example LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# Set C++ standard to C++17 or higher
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose build type: Debug, Release, RelWithDebInfo, MinSizeRel" FORCE)
endif()

# Specify Python version (adjust if needed)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED QUIET)
find_package(Python3 COMPONENTS NumPy QUIET)
find_package(Eigen3 3.4 CONFIG REQUIRED QUIET)
find_package(optimal_parking REQUIRED)
find_package(yaml-cpp REQUIRED)


add_executable(${PROJECT_NAME} to_example.cpp)


target_link_libraries(${PROJECT_NAME}
	Eigen3::Eigen
	Python3::Python
	Python3::Module
	Python3::NumPy
	${YAML_CPP_LIBRARIES}
	optimal_parking
)

# If using matplotlibcpp, ensure that Python is available for linking
if (Python3_FOUND)
    message(STATUS "Python 3 found: ${Python3_VERSION}")
else ()
    message(FATAL_ERROR "Python 3 not found")
endif ()

