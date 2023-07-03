# cmake_minimum_required(VERSION @CMAKE_VERSION@)
cmake_minimum_required(VERSION 3.0.2)
project(@APP_INTERFACE_NAME@)

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
add_message_files(
    FILES
    Num.msg
    # command_t.msg
    # report_t.msg
)
add_service_files(
    FILES
    AddTwoInts.srv
)
add_action_files(
    DIRECTORY action
    FILES 
        Fibonacci.action
        Averaging.action
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
message(STATUS "catkin_INCLUDE_DIRS = ${catkin_INCLUDE_DIRS}")

# ==============================================================================
# C++ Nodes

set(CPP_NODES
    # Topics
    chatter_talker
    chatter_listener
    # Services
    add_two_ints_server
    add_two_ints_client
    # Actions
    fibonacci_server
    fibonacci_client
    fibonacci_client_callback
    fibonacci_client_callback_class
    averaging_server
    averaging_client
    # Linear System Simulator
    # linear_system_simulator_target
)
foreach(NODE ${CPP_NODES})
    add_executable(${NODE}
        src/${NODE}.cpp
    )
    target_link_libraries(${NODE}
        ${catkin_LIBRARIES}
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
    # Topics
    scripts/chatter_talker.py
    scripts/chatter_listener.py
    # Services
    scripts/add_two_ints_server.py
    scripts/add_two_ints_client.py
    # Actions
    scripts/fibonacci_server.py
    scripts/fibonacci_client.py
    scripts/averaging_random_number_generator.py
    # Linear System Simulator
    # scripts/linear_system_simulator_interface.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# ==============================================================================
# Extra Files

install(FILES
    launch/chatter.launch
    launch/chatter_cpp.launch
    launch/chatter_py.launch
    launch/add_two_ints.launch
    launch/add_two_ints_cpp.launch
    launch/add_two_ints_py.launch
    launch/fibonacci_cpp.launch
    launch/fibonacci_py.launch
    launch/fibonacci.launch
    launch/fibonacci_callback.launch
    launch/fibonacci_callback_class.launch
    launch/averaging.launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
    test/chatter
    test/add_two_ints
    test/fibonacci
    test/fibonacci_callback
    test/fibonacci_callback_class
    DESTINATION ${CMAKE_INSTALL_PREFIX}/test
)