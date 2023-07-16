macro(install_tinyxml2)

# given: APP_TINYXML2_INSTALL

if(NOT APP_TINYXML2_INSTALL)
    return()
endif()
set(APP_TINYXML2_NAME tinyxml2)
set(APP_TINYXML2_DIR "${APP_LIB_DIR}/${APP_TINYXML2_NAME}")
if(NOT EXISTS "${APP_TINYXML2_DIR}")
    message(STATUS "NOT found TinyXml2: Start to install...")
    file(DOWNLOAD 
        "https://github.com/leethomason/tinyxml2/archive/refs/heads/master.zip"
        "${APP_BINARY_DIR}/tinyxml2.zip"
        SHOW_PROGRESS
        STATUS EIGEN_DOWNLOAD_STATUS
    )
    file(ARCHIVE_EXTRACT 
        INPUT "${APP_BINARY_DIR}/tinyxml2.zip"
        DESTINATION "${APP_TINYXML2_DIR}"
    )
    file(RENAME "${APP_TINYXML2_DIR}/tinyxml2-master" "${APP_TINYXML2_DIR}/source")
    execute_process(
        COMMAND ${APP_SHELL_EXE} -c "mkdir build && cd build && cmake ../source -DCMAKE_INSTALL_PREFIX=../ && make && make install"
        WORKING_DIRECTORY "${APP_TINYXML2_DIR}"
    )
endif()
if(EXISTS "${APP_TINYXML2_DIR}/include" AND EXISTS "${APP_TINYXML2_DIR}/lib")
    set(APP_TINYXML2_INCLUDE_DIRS "${APP_LIB_DIR}/tinyxml2/include")
    set(APP_TINYXML2_LIBRARY_DIRS "${APP_LIB_DIR}/tinyxml2/lib")
    set(APP_TINYXML2_LIBRARIES tinyxml2)
    message(STATUS ${STRING_REPEAT_MINUS_80})
    message(STATUS "Found TinyXml2")
    message(STATUS "    APP_TINYXML2_NAME = ${APP_TINYXML2_NAME}")
    message(STATUS "    APP_TINYXML2_DIR = ${APP_TINYXML2_DIR}")
    message(STATUS "    APP_TINYXML2_INCLUDE_DIRS = ${APP_TINYXML2_INCLUDE_DIRS}")
    message(STATUS "    APP_TINYXML2_LIBRARY_DIRS = ${APP_TINYXML2_LIBRARY_DIRS}")
    message(STATUS "    APP_TINYXML2_LIBRARIES = ${APP_TINYXML2_LIBRARIES}")
    message(STATUS ${STRING_REPEAT_MINUS_80})
endif()

endmacro()