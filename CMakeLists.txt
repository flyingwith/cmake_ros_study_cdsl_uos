cmake_minimum_required(VERSION 3.19.0)

project(cmake_ros_study)

# ==============================================================================
# setup cmake

set(APP_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/..)
set(APP_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(APP_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR})
set(APP_LIBRARY_DIR ${APP_ROOT_DIR}/library)
set(APP_INSTALL_DIR ${APP_ROOT_DIR}/install)

set(APP_INCLUDE_DIRS "")
set(APP_LIBRARY_DIRS "")
set(APP_LIBRARIES "")
set(APP_COMPILE_OPTIONS
    -std=c++11
    -Wall
    -fPIC
    -Ofast              # Ofast is better than O2 when there are several nested loops
    #-O2
    #-O3
    -DEIGEN_NO_DEBUG
    # -g                  # valgrind
)
set(APP_LINK_OPTIONS
    -std=c++11
)

# ==============================================================================
# Eigen library

set(APP_INCLUDE_DIRS
    "${APP_INCLUDE_DIRS}"
    ${APP_LIBRARY_DIR}/eigen-3.4.0
)

# ==============================================================================
# QuadProg++ library

set(APP_INCLUDE_DIRS
    "${APP_INCLUDE_DIRS}"
    ${APP_LIBRARY_DIR}/QuadProgpp/include/QuadProg++
)
set(APP_LIBRARY_DIRS
    "${APP_LIBRARY_DIRS}"
    ${APP_LIBRARY_DIR}/QuadProgpp/lib
)
set(APP_LIBRARIES
    ${APP_LIBRARIES}
    quadprog
)

# ==============================================================================

# include_directories(${APP_INCLUDE_DIRS})

# ==============================================================================
# simulation

add_subdirectory(sim)

# ==============================================================================
# executable

add_executable(${PROJECT_NAME} main.cpp)
target_include_directories(${PROJECT_NAME}
    PUBLIC "${APP_INCLUDE_DIRS}"
    PUBLIC "${APP_SOURCE_DIR}/sim"
)
target_link_directories(${PROJECT_NAME}
    PUBLIC "${APP_LIBRARY_DIRS}"
    PUBLIC "${APP_BINARY_DIR}/sim"
)
target_link_libraries(${PROJECT_NAME}
    PUBLIC "${APP_LIBRARIES}"
    PUBLIC simulation
)
set_target_properties(${PROJECT_NAME} PROPERTIES
    COMPILE_OPTIONS "${APP_COMPILE_OPTIONS}"
    LINK_OPTIONS "${APP_LINK_OPTIONS}"
)
install(
    TARGETS ${PROJECT_NAME}
    DESTINATION "${APP_INSTALL_DIR}"
)