cmake_minimum_required(VERSION 3.19.0)

project(cmake_ros_study)

set(APP_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/..)
set(APP_INSTALL_DIR ${APP_ROOT_DIR}/install)

add_executable(${PROJECT_NAME} main.cpp)

install(
    TARGETS ${PROJECT_NAME}
    DESTINATION "${APP_INSTALL_DIR}"
)