macro(install_eigen)

# given: APP_EIGEN_VER

string(REPLACE . _ APP_EIGEN_NAME "eigen.${APP_EIGEN_VER}")
set(APP_EIGEN_DIR "${APP_LIB_DIR}/${APP_EIGEN_NAME}")
if(NOT EXISTS "${APP_EIGEN_DIR}")
    message(STATUS "NOT found Eigen ${APP_EIGEN_VER}: Start to install...")
    file(DOWNLOAD 
        "https://gitlab.com/libeigen/eigen/-/archive/${APP_EIGEN_VER}/eigen-${APP_EIGEN_VER}.zip"
        "${APP_BINARY_DIR}/eigen-${APP_EIGEN_VER}.zip"
        SHOW_PROGRESS
        STATUS EIGEN_DOWNLOAD_STATUS
    )
    file(ARCHIVE_EXTRACT 
        INPUT "${APP_BINARY_DIR}/eigen-${APP_EIGEN_VER}.zip"
        DESTINATION "${APP_LIB_DIR}"
    )
    file(RENAME "${APP_LIB_DIR}/eigen-${APP_EIGEN_VER}" "${APP_EIGEN_DIR}")
endif()
if(EXISTS "${APP_EIGEN_DIR}/Eigen")
    set(APP_EIGEN_INCLUDE_DIRS ${APP_EIGEN_DIR})
    set(APP_EIGEN_COMPILE_OPTIONS -DEIGEN_NO_DEBUG)
    message(STATUS ${STRING_REPEAT_MINUS_80})
    message(STATUS "Found Eigen")
    message(STATUS "    APP_EIGEN_VER = ${APP_EIGEN_VER}")
    message(STATUS "    APP_EIGEN_NAME = ${APP_EIGEN_NAME}")
    message(STATUS "    APP_EIGEN_DIR = ${APP_EIGEN_DIR}")
    message(STATUS "    APP_EIGEN_INCLUDE_DIRS = ${APP_EIGEN_INCLUDE_DIRS}")
    message(STATUS "    APP_EIGEN_COMPILE_OPTIONS = ${APP_EIGEN_COMPILE_OPTIONS}")
    message(STATUS ${STRING_REPEAT_MINUS_80})
endif()

endmacro()