# ==============================================================================
# test_xmlreader

configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/parameter.xml"
    "${CMAKE_CURRENT_BINARY_DIR}/parameter.xml"
)
add_executable(${APP_NAME}_tools_test_xmlreader
    "${CMAKE_CURRENT_SOURCE_DIR}/test_xmlreader.cpp"
)
target_include_directories(${APP_NAME}_tools_test_xmlreader
    PUBLIC "${APP_TOOLS_INCLUDE_DIRS}"
)
target_link_directories(${APP_NAME}_tools_test_xmlreader
    PUBLIC "${APP_TOOLS_LIBRARY_DIRS}"
)
target_link_libraries(${APP_NAME}_tools_test_xmlreader
    PUBLIC "${APP_TOOLS_LIBRARIES}"
)
set_target_properties(${APP_NAME}_tools_test_xmlreader PROPERTIES
    COMPILE_OPTIONS "${APP_TOOLS_COMPILE_OPTIONS}"
    LINK_OPTIONS "${APP_TOOLS_LINK_OPTIONS}"
)
install(
    TARGETS ${APP_NAME}_tools_test_xmlreader
    DESTINATION "${APP_TOOLS_INSTALL_DIR}/test_xmlreader"
)
install(
    FILES "${CMAKE_CURRENT_BINARY_DIR}/parameter.xml"
    DESTINATION "${APP_TOOLS_INSTALL_DIR}/test_xmlreader"
)

# ==============================================================================
# test_py_xmlreader

configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/test_py_xmlreader.py"
    "${CMAKE_CURRENT_BINARY_DIR}/test_py_xmlreader.py"
)
execute_process(
    COMMAND 
        ${APP_CYTHON_EXE}
        test_py_xmlreader.py # source
        -I${CMAKE_CURRENT_SOURCE_DIR} # include directory
        -3 # uses python 3
        --cplus # generates .cpp
        --annotate # generates .html
        --embed # generates main() function
    WORKING_DIRECTORY
        "${CMAKE_CURRENT_BINARY_DIR}"
)
add_executable(${APP_NAME}_tools_test_py_xmlreader
    "${CMAKE_CURRENT_BINARY_DIR}/test_py_xmlreader.cpp"
)
target_include_directories(${APP_NAME}_tools_test_py_xmlreader
    PUBLIC "${APP_TOOLS_INCLUDE_DIRS}"
)
target_link_directories(${APP_NAME}_tools_test_py_xmlreader
    PUBLIC "${APP_TOOLS_LIBRARY_DIRS}"
)
target_link_libraries(${APP_NAME}_tools_test_py_xmlreader 
    PUBLIC "${APP_TOOLS_LIBRARIES}"
)
set_target_properties(${APP_NAME}_tools_test_py_xmlreader PROPERTIES 
    COMPILE_OPTIONS "${APP_TOOLS_COMPILE_OPTIONS}"
    LINK_OPTIONS "${APP_TOOLS_LINK_OPTIONS}"
)
install(
    TARGETS ${APP_NAME}_tools_test_py_xmlreader
    DESTINATION "${APP_TOOLS_INSTALL_DIR}/test_xmlreader"
)

# ========================================================================================
# test script

if(CMAKE_SYSTEM_NAME MATCHES "Linux" OR CMAKE_SYSTEM_NAME MATCHES "Darwin")
    set(TEST_SCRIPT_NAME "start_test")
    file(
        WRITE ${CMAKE_CURRENT_BINARY_DIR}/${TEST_SCRIPT_NAME}
        "cd $(dirname \"$0\")\n"
        "read -n 1 -s -r -p \"\n[Press any key to run test_xmlreader]\n\n\"\n"
        "./${APP_NAME}_tools_test_xmlreader\n"
        # "read -n 1 -s -r -p \"\n[Press any key to run test_xmlreader with valgrind]\n\n\"\n"
        # "valgrind --leak-check=yes --track-origins=yes -v ./test_xmlreader"
        "read -n 1 -s -r -p \"\n[Press any key to run test_py_xmlreader]\n\n\"\n"
        "./${APP_NAME}_tools_test_py_xmlreader\n"
    )
    execute_process(COMMAND chmod +x ${TEST_SCRIPT_NAME} WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}")
elseif(CMAKE_SYSTEM_NAME MATCHES "Windows")
else()
endif()

install(
    PROGRAMS "${CMAKE_CURRENT_BINARY_DIR}/${TEST_SCRIPT_NAME}"
    DESTINATION "${APP_TOOLS_INSTALL_DIR}/test_xmlreader"
)