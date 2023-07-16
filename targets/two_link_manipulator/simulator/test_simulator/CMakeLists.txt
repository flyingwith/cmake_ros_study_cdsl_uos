# ==============================================================================
# test_simulator

configure_file(
    "${TARGET_SOURCE_DIR}/parameter.xml"
    "${CMAKE_CURRENT_BINARY_DIR}/parameter.xml"
)
add_executable(${TARGET_NAME}_test_simulator
    "${CMAKE_CURRENT_SOURCE_DIR}/test_simulator.cpp"
)
target_include_directories(${TARGET_NAME}_test_simulator
    PUBLIC "${TARGET_INCLUDE_DIRS}"
    PUBLIC "${TARGET_SOURCE_DIR}/simulator"
)
target_link_directories(${TARGET_NAME}_test_simulator
    PUBLIC "${TARGET_LIBRARY_DIRS}"
    PUBLIC "${TARGET_BINARY_DIR}/simulator"
)
target_link_libraries(${TARGET_NAME}_test_simulator
    PUBLIC "${TARGET_LIBRARIES}"
    PUBLIC "${TARGET_NAME}_simulation1"
)
set_target_properties(${TARGET_NAME}_test_simulator PROPERTIES
    COMPILE_OPTIONS "${TARGET_COMPILE_OPTIONS}"
    LINK_OPTIONS "${TARGET_LINK_OPTIONS}"
)
install(
    TARGETS ${TARGET_NAME}_test_simulator
    DESTINATION "${TARGET_INSTALL_DIR}/simulator/test_simulator"
)
install(
    FILES "${CMAKE_CURRENT_BINARY_DIR}/parameter.xml"
    DESTINATION "${TARGET_INSTALL_DIR}/simulator/test_simulator"
)

# ========================================================================================
# test script

if(CMAKE_SYSTEM_NAME MATCHES "Linux" OR CMAKE_SYSTEM_NAME MATCHES "Darwin")
    set(TEST_SCRIPT_NAME "start_test")
    file(
        WRITE ${CMAKE_CURRENT_BINARY_DIR}/${TEST_SCRIPT_NAME}
        "cd $(dirname \"$0\")\n"
        "read -n 1 -s -r -p \"\n[Press any key to run test_simulator]\n\n\"\n"
        "./${TARGET_NAME}_test_simulator\n"
        # "read -n 1 -s -r -p \"\n[Press any key to run test_simulator with valgrind]\n\n\"\n"
        # "valgrind --leak-check=yes --track-origins=yes -v ./test_simulator"
    )
    execute_process(COMMAND chmod +x ${TEST_SCRIPT_NAME} WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}")
elseif(CMAKE_SYSTEM_NAME MATCHES "Windows")
else()
endif()

install(
    PROGRAMS "${CMAKE_CURRENT_BINARY_DIR}/${TEST_SCRIPT_NAME}"
    DESTINATION "${TARGET_INSTALL_DIR}/simulator/test_simulator"
)