macro(install_quadprogpp)

# given: APP_QUADPROGPP_INSTALL

if(NOT APP_QUADPROGPP_INSTALL)
    return()
endif()
set(APP_QUADPROGPP_NAME quadprogpp)
set(APP_QUADPROGPP_DIR "${APP_LIB_DIR}/${APP_QUADPROGPP_NAME}")
if(NOT EXISTS "${APP_QUADPROGPP_DIR}")
    message(STATUS "NOT found QuadProg++: Start to install...")
    file(DOWNLOAD 
        "https://github.com/liuq/QuadProgpp/archive/refs/heads/master.zip"
        "${APP_BINARY_DIR}/quadprogpp.zip"
        SHOW_PROGRESS
        STATUS EIGEN_DOWNLOAD_STATUS
    )
    file(ARCHIVE_EXTRACT 
        INPUT "${APP_BINARY_DIR}/quadprogpp.zip"
        DESTINATION "${APP_QUADPROGPP_DIR}"
    )
    file(RENAME "${APP_QUADPROGPP_DIR}/QuadProgpp-master" "${APP_QUADPROGPP_DIR}/source")
    execute_process(
        COMMAND ${APP_SHELL_EXE} -c "mkdir build && cd build && cmake ../source -DCMAKE_INSTALL_PREFIX=../ && make && make install"
        WORKING_DIRECTORY "${APP_QUADPROGPP_DIR}"
    )
endif()
if(EXISTS "${APP_QUADPROGPP_DIR}/include" AND EXISTS "${APP_QUADPROGPP_DIR}/lib")
    set(APP_QUADPROGPP_INCLUDE_DIRS "${APP_LIB_DIR}/quadprogpp/include/QuadProg++")
    set(APP_QUADPROGPP_LIBRARY_DIRS "${APP_LIB_DIR}/quadprogpp/lib")
    set(APP_QUADPROGPP_LIBRARIES quadprog)
    message(STATUS ${STRING_REPEAT_MINUS_80})
    message(STATUS "Found QuadProg++")
    message(STATUS "    APP_QUADPROGPP_NAME = ${APP_QUADPROGPP_NAME}")
    message(STATUS "    APP_QUADPROGPP_DIR = ${APP_QUADPROGPP_DIR}")
    message(STATUS "    APP_QUADPROGPP_INCLUDE_DIRS = ${APP_QUADPROGPP_INCLUDE_DIRS}")
    message(STATUS "    APP_QUADPROGPP_LIBRARY_DIRS = ${APP_QUADPROGPP_LIBRARY_DIRS}")
    message(STATUS "    APP_QUADPROGPP_LIBRARIES = ${APP_QUADPROGPP_LIBRARIES}")
    message(STATUS ${STRING_REPEAT_MINUS_80})
endif()

endmacro()