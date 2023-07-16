cmake_minimum_required(VERSION 3.0.2)
project(@TARGET_ROS_NAME@)

# ==============================================================================
# ROS Settings

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation
    actionlib
    actionlib_msgs
)

# add_message_files(
#     FILES
# )

# add_service_files(
#     FILES
# )

add_action_files(
    DIRECTORY action
    FILES 
    simulation1.action
)

generate_messages(
    DEPENDENCIES
    std_msgs
    actionlib_msgs
)

catkin_package(
    CATKIN_DEPENDS 
    message_runtime 
    roscpp 
    rospy 
    std_msgs
    actionlib
    actionlib_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

# ==============================================================================
# C++ Nodes

set(CPP_NODES
    simulation1_server
)
foreach(NODE ${CPP_NODES})
    add_executable(${NODE}
        src/${NODE}.cpp
    )
    target_include_directories(${NODE}
        PUBLIC "@TARGET_ROS_INCLUDE_DIRS@"
    )
    target_link_directories(${NODE}
        PUBLIC "@TARGET_ROS_LIBRARY_DIRS@"
    )
    target_link_libraries(${NODE}
        PUBLIC "@TARGET_ROS_LIBRARIES@"
        PUBLIC ${catkin_LIBRARIES}
    )
    set_target_properties(${NODE} PROPERTIES
        COMPILE_OPTIONS "@TARGET_ROS_COMPILE_OPTIONS@"
        LINK_OPTIONS "@TARGET_ROS_LINK_OPTIONS@"
    )
    add_dependencies(${NODE}
        ${PROJECT_NAME}_generate_messages_cpp
    )
    install(
        TARGETS ${NODE}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
endforeach()

# ==============================================================================
# Python Nodes

catkin_install_python(PROGRAMS
    scripts/simulation1_client.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# ==============================================================================
# Extra Files

install(FILES
    launch/simulation1_test.launch
    launch/simulation1_server.launch
    launch/simulation1_client.launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
    test/simulation1_test
    test/simulation1_server
    test/simulation1_client
    test/simulation1_close
    DESTINATION ${CMAKE_INSTALL_PREFIX}/test
)