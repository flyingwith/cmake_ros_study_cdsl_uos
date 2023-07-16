# ==============================================================================
# test_interface

configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/test_interface.py"
    "${CMAKE_CURRENT_BINARY_DIR}/test_interface.py"
)
execute_process(
    COMMAND 
        ${APP_CYTHON_EXE}
        test_interface.py # source
        -I${CMAKE_CURRENT_SOURCE_DIR} # include directory
        -3 # uses python 3
        --cplus # generates .cpp
        --annotate # generates .html
        --embed # generates main() function
    WORKING_DIRECTORY
        "${CMAKE_CURRENT_BINARY_DIR}"
)
add_executable(${APP_NAME}_test_interface
    "${CMAKE_CURRENT_BINARY_DIR}/test_interface.cpp"
)
target_include_directories(${APP_NAME}_test_interface
    PUBLIC "${APP_TOOLS_INCLUDE_DIRS}"
)
target_link_directories(${APP_NAME}_test_interface
    PUBLIC "${APP_TOOLS_LIBRARY_DIRS}"
)
target_link_libraries(${APP_NAME}_test_interface 
    PUBLIC "${APP_TOOLS_LIBRARIES}"
)
set_target_properties(${APP_NAME}_test_interface PROPERTIES 
    COMPILE_OPTIONS "${APP_TOOLS_COMPILE_OPTIONS}"
    LINK_OPTIONS "${APP_TOOLS_LINK_OPTIONS}"
)
install(
    TARGETS ${APP_NAME}_test_interface
    DESTINATION "${APP_INTERFACE_INSTALL_DIR}/test_interface"
)

# ========================================================================================
# test script

if(CMAKE_SYSTEM_NAME MATCHES "Darwin")
    file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/${APP_NAME}_test_interface_run
"\
cd $(dirname \"$0\")
read -n 1 -s -r -p \"\n[Press any key to run test_interface]\n\n\"
./${APP_NAME}_test_interface
"
    )
    execute_process(
        COMMAND chmod +x ${APP_NAME}_test_interface_run 
        WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
    )
    
    set(TEST_SCRIPT_NAME "start_test")
    file(
        WRITE "${CMAKE_CURRENT_BINARY_DIR}/start_test"
"\
#!${APP_SHELL_EXE}
osascript - \"$(dirname \"$0\")\" << EOF
on run argv -- argv is a list of string
    tell application \"Terminal\"
        do script (\"cd \" & quoted form of item 1 of argv & \"/../../scripts && ./run_roscore\")
        do script (\"cd \" & quoted form of item 1 of argv & \" && ./${APP_NAME}_test_interface_run\")
    end tell
end run
EOF
"
    )
    execute_process(
        COMMAND chmod +x start_test 
        WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
    )
elseif(CMAKE_SYSTEM_NAME MATCHES "Linux")
elseif(CMAKE_SYSTEM_NAME MATCHES "Windows")
else()
endif()

install(
    PROGRAMS "${CMAKE_CURRENT_BINARY_DIR}/${APP_NAME}_test_interface_run"
    DESTINATION "${APP_INTERFACE_INSTALL_DIR}/test_interface"
)
install(
    PROGRAMS "${CMAKE_CURRENT_BINARY_DIR}/start_test"
    DESTINATION "${APP_INTERFACE_INSTALL_DIR}/test_interface"
)