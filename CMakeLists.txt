cmake_minimum_required(VERSION 3.19.2)

project(cmake_ros_study VERSION 0.0.0)

string(REPEAT "=" 80 STRING_REPEAT_EQUAL_80)
string(REPEAT "-" 80 STRING_REPEAT_MINUS_80)

message(STATUS "${STRING_REPEAT_EQUAL_80}")
message(STATUS "CMake Configuration of ${PROJECT_NAME} Version ${PROJECT_VERSION}")
message(STATUS "${STRING_REPEAT_EQUAL_80}")

# ==============================================================================
# Project Parameters

set(APP_EIGEN_VER 3.4.0)
set(APP_QUADPROGPP_INSTALL True) # True / False
set(APP_CONDA_ENV_VER ros.noetic) # python.3.9 / python.3.10 / ros.noetic / ros.humble
set(APP_MAMBA_DIR /Users/sangik/mambaforge)
set(APP_SHELL_TYPE zsh) # bash / zsh
set(APP_SHELL_EXE /bin/${APP_SHELL_TYPE})
set(APP_TARGETS # TEXT_NAME DIR_NAME
    "Linear System Simulator" "linear_system_simulator"
    "Two-Link Manipulator" "two_link_manipulator"
    "Franka Emika Panda" "franka_emika_panda"
    "KIST Mujoco Simulator" "kist_mujoco_simulator"
)

# ==============================================================================
# Project Setup

set(APP_NAME ${PROJECT_NAME})
string(TOLOWER "${APP_NAME}" APP_NAME_LOWERCASE)
set(APP_VER ${PROJECT_VERSION})
string(REPLACE . _ APP_VER_ ${APP_VER})
get_filename_component(APP_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}" DIRECTORY)
set(APP_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(APP_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR})
set(APP_LIB_DIR ${APP_ROOT_DIR}/library)
set(APP_INSTALL_DIR ${APP_ROOT_DIR}/install)
set(APP_DATA_DIR ${APP_ROOT_DIR}/data)
set(APP_CONFIG_DIR ${APP_SOURCE_DIR}/config)
set(APP_CONDA_DIR "${APP_MAMBA_DIR}")

configure_file(
    "${APP_CONFIG_DIR}/app.h"
    "${APP_BINARY_DIR}/app.h"
)

set(APP_INCLUDE_DIRS "${APP_BINARY_DIR}")
set(APP_LIBRARY_DIRS "")
set(APP_LIBRARIES "")
set(APP_COMPILE_OPTIONS
    -std=c++11
    -Wall
    -fPIC
    -Ofast
)
set(APP_LINK_OPTIONS
    -std=c++11
)

# ==============================================================================
# CMake Setup

# C++ version
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# path to additional CMake config files
list(APPEND CMAKE_MODULE_PATH "${APP_CONFIG_DIR}")

# ==============================================================================
# Eigen

include("${APP_CONFIG_DIR}/Eigen.cmake")
install_eigen()

# ==============================================================================
# QuadProg++

include("${APP_CONFIG_DIR}/QuadProgPP.cmake")
install_quadprogpp()

# ==============================================================================
# Conda Environment with/without ROS

include("${APP_CONFIG_DIR}/CondaEnv.cmake")
install_conda_env()

# ==============================================================================
# Targets

include("${APP_CONFIG_DIR}/Targets.cmake")
add_targets()
add_subdirectory(targets)

# ==============================================================================
# Interface

add_subdirectory(interface)