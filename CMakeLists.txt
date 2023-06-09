cmake_minimum_required(VERSION 3.19.0)

project(cmake_ros_study)

# ==============================================================================
# setup cmake

set(APP_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/..)
set(APP_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(APP_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR})
set(APP_LIBRARY_DIR ${APP_ROOT_DIR}/library)
set(APP_INSTALL_DIR ${APP_ROOT_DIR}/install)

# ==============================================================================
# library

add_subdirectory(sim)

# include_directories(${APP_SOURCE_DIR}/sim)

# ==============================================================================
# executable

add_executable(${PROJECT_NAME} main.cpp)
target_include_directories(${PROJECT_NAME}
    PUBLIC "${APP_SOURCE_DIR}/sim"
)
target_link_directories(${PROJECT_NAME}
    PUBLIC "${APP_BINARY_DIR}/sim"
)
target_link_libraries(${PROJECT_NAME}
    PUBLIC simulation
)
install(
    TARGETS ${PROJECT_NAME}
    DESTINATION "${APP_INSTALL_DIR}"
)