macro(install_tools)

set(APP_TOOLS_SOURCE_DIR "${APP_SOURCE_DIR}/tools")
set(APP_TOOLS_BINARY_DIR "${APP_BINARY_DIR}/tools")
set(APP_TOOLS_INSTALL_DIR "${APP_LIB_DIR}/tools")
set(APP_TOOLS_INCLUDE_DIRS
    ${APP_INCLUDE_DIRS}
    ${APP_EIGEN_INCLUDE_DIRS}
    ${APP_TINYXML2_INCLUDE_DIRS}
    ${APP_QUADPROGPP_INCLUDE_DIRS}
    ${APP_PYTHON_INCLUDE_DIRS}
    ${APP_TOOLS_SOURCE_DIR}
)
set(APP_TOOLS_LIBRARY_DIRS
    ${APP_LIBRARY_DIRS}
    ${APP_TINYXML2_LIBRARY_DIRS}
    ${APP_QUADPROGPP_LIBRARY_DIRS}
    ${APP_PYTHON_LIBRARY_DIRS}
    ${APP_TOOLS_INSTALL_DIR}
)
set(APP_TOOLS_LIBRARIES
    ${APP_LIBRARIES}
    ${APP_TINYXML2_LIBRARIES}
    ${APP_QUADPROGPP_LIBRARIES}
    ${APP_PYTHON_LIBRARIES}
)
set(APP_TOOLS_COMPILE_OPTIONS
    ${APP_COMPILE_OPTIONS}
    ${APP_EIGEN_COMPILE_OPTIONS}
)
set(APP_TOOLS_LINK_OPTIONS
    ${APP_LINK_OPTIONS}
    ${APP_PYTHON_LINK_OPTIONS}
)
if(NOT EXISTS "${APP_TOOLS_INSTALL_DIR}" OR APP_TOOLS_BUILD_ALWAYS)
    add_subdirectory("${APP_TOOLS_SOURCE_DIR}" "${APP_TOOLS_BINARY_DIR}")
endif()
if(EXISTS "${APP_TOOLS_INSTALL_DIR}")
    message(STATUS ${STRING_REPEAT_MINUS_80})
    message(STATUS "Found tools")
    message(STATUS "    APP_TOOLS_SOURCE_DIR = ${APP_TOOLS_SOURCE_DIR}")
    message(STATUS "    APP_TOOLS_BINARY_DIR = ${APP_TOOLS_BINARY_DIR}")
    message(STATUS "    APP_TOOLS_INSTALL_DIR = ${APP_TOOLS_INSTALL_DIR}")
    message(STATUS "    APP_TOOLS_INCLUDE_DIRS = ${APP_TOOLS_INCLUDE_DIRS}")
    message(STATUS "    APP_TOOLS_LIBRARY_DIRS = ${APP_TOOLS_LIBRARY_DIRS}")
    message(STATUS "    APP_TOOLS_LIBRARIES = ${APP_TOOLS_LIBRARIES}")
    message(STATUS "    APP_TOOLS_COMPILE_OPTIONS = ${APP_TOOLS_COMPILE_OPTIONS}")
    message(STATUS "    APP_TOOLS_LINK_OPTIONS = ${APP_TOOLS_LINK_OPTIONS}")
    message(STATUS ${STRING_REPEAT_MINUS_80})
endif()

endmacro()