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
set(APP_TINYXML2_INSTALL True) # True / False
set(APP_QUADPROGPP_INSTALL True) # True / False
set(APP_CONDA_ENV_VER ros.noetic) # python.3.9 / python.3.10 / ros.noetic / ros.humble
set(APP_TOOLS_BUILD_ALWAYS True) # True / False
set(APP_MAMBA_DIR /Users/sangik/mambaforge)
set(APP_SHELL_TYPE zsh) # bash / zsh
set(APP_SHELL_EXE /bin/${APP_SHELL_TYPE})
set(APP_TARGETS # TEXT_NAME DIR_NAME
    "Linear System Simulator" "linear_system_simulator"
    "Two-Link Manipulator" "two_link_manipulator"
    # "Franka Emika Panda" "franka_emika_panda"
    # "KIST Mujoco Simulator" "kist_mujoco_simulator"
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

# The default settings of CMake set the path to the libraries 'absolute' in the  build directory but 'relative' in the install directory. 
# It causes that an excutable runs in the build directory but does not in the install directory.
# To solve this problem, the next post recommends the following settings:
#     https://gitlab.kitware.com/cmake/community/-/wikis/doc/cmake/RPATH-handling
set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
if("${isSystemDir}" STREQUAL "-1")
    set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
endif("${isSystemDir}" STREQUAL "-1")

# ==============================================================================
# Eigen

include("${APP_CONFIG_DIR}/Eigen.cmake")
install_eigen()

# ==============================================================================
# TinyXml2

include("${APP_CONFIG_DIR}/TinyXml2.cmake")
install_tinyxml2()

# ==============================================================================
# QuadProg++

# Caution: QusdProg++ does not work well with Eigen!!!
# - https://stackoverflow.com/questions/53386048/c-compilling-errors-when-using-quadprog-with-eigen-together

# include("${APP_CONFIG_DIR}/QuadProgPP.cmake")
# install_quadprogpp()

# ==============================================================================
# Conda Environment with/without ROS

include("${APP_CONFIG_DIR}/CondaEnv.cmake")
install_conda_env()

# ==============================================================================
# Tools

include("${APP_CONFIG_DIR}/Tools.cmake")
install_tools()

# ==============================================================================
# Shell Scripts

include("${APP_CONFIG_DIR}/ShellScripts.cmake")
generate_shell_scripts()

# ==============================================================================
# Targets

include("${APP_CONFIG_DIR}/Targets.cmake")
add_targets()
add_subdirectory(targets)

# ==============================================================================
# Interface

add_subdirectory(interface)

# ==============================================================================
# Launch Script

if(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
    set(APP_SCRIPT_NAME "${APP_NAME}")
    file(WRITE ${APP_BINARY_DIR}/${APP_SCRIPT_NAME}
"\
#!${APP_SHELL_EXE}
osascript - \"$(dirname \"$0\")\" << EOF
on run argv -- argv is a list of string
    tell application \"Terminal\"
        do script (\"cd \" & quoted form of item 1 of argv & \"/scripts && ./run_roscore\")
        do script (\"cd \" & quoted form of item 1 of argv & \"/interface && ./interface\")
    end tell
end run
EOF
"
    )
    execute_process(
        COMMAND chmod +x ${APP_SCRIPT_NAME} 
        WORKING_DIRECTORY "${APP_BINARY_DIR}"
    )
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux")
elseif(CMAKE_SYSTEM_NAME STREQUAL "Windows")
else()
endif()

install(
    PROGRAMS "${APP_BINARY_DIR}/${APP_SCRIPT_NAME}"
    DESTINATION "${APP_INSTALL_DIR}"
)