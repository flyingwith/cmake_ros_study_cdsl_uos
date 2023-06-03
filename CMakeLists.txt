cmake_minimum_required(VERSION 3.19.0)

project(cmake_ros_study)

# ==============================================================================
# setup cmake

set(APP_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/..)
set(APP_LIBRARY_DIR ${APP_ROOT_DIR}/library)
set(APP_INSTALL_DIR ${APP_ROOT_DIR}/install)

# ==============================================================================
# library

add_library(simulation simulation.cpp)
install(
    FILES simulation.h
    DESTINATION "${APP_LIBRARY_DIR}/include/simulation"
)
install(
    TARGETS simulation
    DESTINATION "${APP_LIBRARY_DIR}/lib"
)

# ==============================================================================
# executable

add_executable(${PROJECT_NAME} main.cpp)
target_include_directories(${PROJECT_NAME}
    PUBLIC "${APP_LIBRARY_DIR}/include/simulation"
)
target_link_directories(${PROJECT_NAME}
    PUBLIC "${APP_LIBRARY_DIR}/lib"
)
target_link_libraries(${PROJECT_NAME}
    PUBLIC simulation
)
install(
    TARGETS ${PROJECT_NAME}
    DESTINATION "${APP_INSTALL_DIR}"
)