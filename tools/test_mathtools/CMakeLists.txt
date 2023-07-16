# ==============================================================================
# test_mathtools

add_executable(${APP_NAME}_tools_test_mathtools
    "${CMAKE_CURRENT_SOURCE_DIR}/test_mathtools.cpp"
)
target_include_directories(${APP_NAME}_tools_test_mathtools
    PUBLIC "${APP_TOOLS_INCLUDE_DIRS}"
)
target_link_directories(${APP_NAME}_tools_test_mathtools
    PUBLIC "${APP_TOOLS_LIBRARY_DIRS}"
)
target_link_libraries(${APP_NAME}_tools_test_mathtools
    PUBLIC "${APP_TOOLS_LIBRARIES}"
)
set_target_properties(${APP_NAME}_tools_test_mathtools PROPERTIES
    COMPILE_OPTIONS "${APP_TOOLS_COMPILE_OPTIONS}"
    LINK_OPTIONS "${APP_TOOLS_LINK_OPTIONS}"
)
install(
    TARGETS ${APP_NAME}_tools_test_mathtools
    DESTINATION "${APP_TOOLS_INSTALL_DIR}/test_mathtools"
)

# ==============================================================================
# test_py_mathtools

configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/test_py_mathtools.py"
    "${CMAKE_CURRENT_BINARY_DIR}/test_py_mathtools.py"
)
execute_process(
    COMMAND 
        ${APP_CYTHON_EXE}
        test_py_mathtools.py # source
        -I${CMAKE_CURRENT_SOURCE_DIR} # include directory
        -3 # uses python 3
        --cplus # generates .cpp
        --annotate # generates .html
        --embed # generates main() function
    WORKING_DIRECTORY
        "${CMAKE_CURRENT_BINARY_DIR}"
)
add_executable(${APP_NAME}_tools_test_py_mathtools
    "${CMAKE_CURRENT_BINARY_DIR}/test_py_mathtools.cpp"
)
target_include_directories(${APP_NAME}_tools_test_py_mathtools
    PUBLIC "${APP_TOOLS_INCLUDE_DIRS}"
)
target_link_directories(${APP_NAME}_tools_test_py_mathtools
    PUBLIC "${APP_TOOLS_LIBRARY_DIRS}"
)
target_link_libraries(${APP_NAME}_tools_test_py_mathtools 
    PUBLIC "${APP_TOOLS_LIBRARIES}"
)
set_target_properties(${APP_NAME}_tools_test_py_mathtools PROPERTIES 
    COMPILE_OPTIONS "${APP_TOOLS_COMPILE_OPTIONS}"
    LINK_OPTIONS "${APP_TOOLS_LINK_OPTIONS}"
)
install(
    TARGETS ${APP_NAME}_tools_test_py_mathtools
    DESTINATION "${APP_TOOLS_INSTALL_DIR}/test_mathtools"
)

# ========================================================================================
# test script

if(CMAKE_SYSTEM_NAME MATCHES "Linux" OR CMAKE_SYSTEM_NAME MATCHES "Darwin")
    set(TEST_SCRIPT_NAME "start_test")
    file(
        WRITE ${CMAKE_CURRENT_BINARY_DIR}/${TEST_SCRIPT_NAME}
        "cd $(dirname \"$0\")\n"
        "read -n 1 -s -r -p \"\n[Press any key to run test_mathtools]\n\n\"\n"
        "./${APP_NAME}_tools_test_mathtools\n"
        # "read -n 1 -s -r -p \"\n[Press any key to run test_mathtools with valgrind]\n\n\"\n"
        # "valgrind --leak-check=yes --track-origins=yes -v ./test_mathtools"
        "read -n 1 -s -r -p \"\n[Press any key to run test_py_mathtools]\n\n\"\n"
        "./${APP_NAME}_tools_test_py_mathtools\n"
    )
    execute_process(COMMAND chmod +x ${TEST_SCRIPT_NAME} WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}")
elseif(CMAKE_SYSTEM_NAME MATCHES "Windows")
else()
endif()

install(
    PROGRAMS "${CMAKE_CURRENT_BINARY_DIR}/${TEST_SCRIPT_NAME}"
    DESTINATION "${APP_TOOLS_INSTALL_DIR}/test_mathtools"
)